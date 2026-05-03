UVMS Project Documentation
==========================

.. image:: graphic_abstract_2.jpg
   :alt: UVMS simulator and SimLab overview graphic
   :align: center
   :width: 100%

This documentation covers the underwater vehicle-manipulator simulator and the
SimLab runtime utilities as one ROS 2 UVMS stack. The stack combines simulator
hardware interfaces, robot descriptions, controllers, planners, replay tools,
sensors, perception-facing camera streams, logging, and hardware-in-the-loop
procedures.

Start with :doc:`simlab_overview` for the project map, then
:doc:`installation` and :doc:`userdoc` for setup and first launch. The middle
guides describe runtime operation. The final guides cover sensors, perception
streams, and extension work.

.. toctree::
   :maxdepth: 2
   :caption: Guides

   simlab_overview
   installation
   userdoc
   hil_setup
   services_and_interfaces
   controls_and_menus
   replay_and_experiments
   camera_and_perception
   developer_guide
   license

.. toctree::
   :maxdepth: 1
   :caption: Project Links

   GitHub: uvms-simulator <https://github.com/edxmorgan/uvms-simulator>
   GitHub: uvms-simlab <https://github.com/edxmorgan/uvms-simlab>
   GitHub: Floating-KinDyn-Graph <https://github.com/edxmorgan/Floating-KinDyn-Graph>
   GitHub: diff_uv <https://github.com/edxmorgan/diff_uv>
