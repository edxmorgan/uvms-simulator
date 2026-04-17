import argparse
import threading
from typing import Dict

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class SimResetCoordinator(Node):
    def __init__(self, robot_prefixes: list[str]) -> None:
        super().__init__("sim_reset_coordinator")
        self._callback_group = ReentrantCallbackGroup()
        self._robot_prefixes = [prefix.strip() for prefix in robot_prefixes if prefix.strip()]

        self._reset_vehicle_clients: Dict[str, rclpy.client.Client] = {}
        self._reset_manipulator_clients: Dict[str, rclpy.client.Client] = {}
        self._release_vehicle_clients: Dict[str, rclpy.client.Client] = {}
        self._release_manipulator_clients: Dict[str, rclpy.client.Client] = {}
        self._reset_services = []
        self._release_services = []

        for prefix in self._robot_prefixes:
            self._reset_vehicle_clients[prefix] = self.create_client(
                Trigger, f"/{prefix}reset_sim_vehicle", callback_group=self._callback_group
            )
            self._reset_manipulator_clients[prefix] = self.create_client(
                Trigger, f"/{prefix}reset_sim_manipulator", callback_group=self._callback_group
            )
            self._release_vehicle_clients[prefix] = self.create_client(
                Trigger, f"/{prefix}release_sim_vehicle", callback_group=self._callback_group
            )
            self._release_manipulator_clients[prefix] = self.create_client(
                Trigger, f"/{prefix}release_sim_manipulator", callback_group=self._callback_group
            )

            self._reset_services.append(
                self.create_service(
                    Trigger,
                    f"/{prefix}reset_sim_uvms",
                    lambda request, response, robot_prefix=prefix: self._handle_reset(robot_prefix, response),
                    callback_group=self._callback_group,
                )
            )
            self._release_services.append(
                self.create_service(
                    Trigger,
                    f"/{prefix}release_sim_uvms",
                    lambda request, response, robot_prefix=prefix: self._handle_release(robot_prefix, response),
                    callback_group=self._callback_group,
                )
            )

        robots = ", ".join(self._robot_prefixes) if self._robot_prefixes else "<none>"
        self.get_logger().info(f"sim reset coordinator ready for robots: {robots}")

    def _wait_for_service(self, client, service_name: str, timeout_sec: float = 5.0) -> tuple[bool, str]:
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True, ""
        return False, f"timeout waiting for {service_name}"

    def _call_trigger(self, client, service_name: str, timeout_sec: float = 5.0) -> tuple[bool, str]:
        ok, message = self._wait_for_service(client, service_name, timeout_sec=timeout_sec)
        if not ok:
            return False, message

        done_event = threading.Event()
        future = client.call_async(Trigger.Request())

        def _done_callback(_future):
            done_event.set()

        future.add_done_callback(_done_callback)
        if not done_event.wait(timeout=timeout_sec):
            return False, f"timeout calling {service_name}"

        if future.cancelled():
            return False, f"call cancelled for {service_name}"
        if future.exception() is not None:
            return False, f"{service_name} failed: {future.exception()}"

        result = future.result()
        if result is None:
            return False, f"{service_name} returned no response"
        return bool(result.success), result.message

    def _handle_reset(self, prefix: str, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info(f"[{prefix}] combined reset requested")
        steps = [
            (self._reset_manipulator_clients[prefix], f"/{prefix}reset_sim_manipulator"),
            (self._reset_vehicle_clients[prefix], f"/{prefix}reset_sim_vehicle"),
        ]

        messages = []
        for client, service_name in steps:
            ok, message = self._call_trigger(client, service_name)
            messages.append(f"{service_name}: {message or ('ok' if ok else 'failed')}")
            if not ok:
                response.success = False
                response.message = "; ".join(messages)
                self.get_logger().error(f"[{prefix}] combined reset failed: {response.message}")
                return response

        response.success = True
        response.message = "; ".join(messages)
        self.get_logger().info(f"[{prefix}] combined reset completed")
        return response

    def _handle_release(self, prefix: str, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info(f"[{prefix}] combined release requested")
        steps = [
            (self._release_manipulator_clients[prefix], f"/{prefix}release_sim_manipulator"),
            (self._release_vehicle_clients[prefix], f"/{prefix}release_sim_vehicle"),
        ]

        messages = []
        for client, service_name in steps:
            ok, message = self._call_trigger(client, service_name)
            messages.append(f"{service_name}: {message or ('ok' if ok else 'failed')}")
            if not ok:
                response.success = False
                response.message = "; ".join(messages)
                self.get_logger().error(f"[{prefix}] combined release failed: {response.message}")
                return response

        response.success = True
        response.message = "; ".join(messages)
        self.get_logger().info(f"[{prefix}] combined release completed")
        return response


def main() -> None:
    parser = argparse.ArgumentParser(description="Combined reset/release coordinator for simulated UVMS robots.")
    parser.add_argument("--robots-prefix", default="", help="Comma-separated list of robot prefixes.")
    args = parser.parse_args()

    prefixes = [prefix.strip() for prefix in args.robots_prefix.split(",") if prefix.strip()]

    rclpy.init()
    node = SimResetCoordinator(prefixes)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
