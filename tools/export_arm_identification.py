#!/usr/bin/env python3
"""Export weak-form arm regressors from the deployed dynamics-library regressor."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import casadi as ca

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('regressor', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
f = ca.Function.load(str(args.regressor))
q, v, a = [ca.SX.sym(name, 4) for name in ('q', 'v', 'a')]
g = ca.SX.sym('gravity', 3)
base, world = [ca.SX.sym(name, 6) for name in ('base', 'world')]
y = f(q, v, a, g, base, world)
assert y.shape == (4, 48)
# Y = A(q) a + b(q,v). Integrate d(A v)/dt by parts, avoiding measured acceleration.
b = ca.substitute(y, a, ca.SX.zeros(4))
momentum = ca.SX.zeros(4, 48)
for j in range(4):
    momentum += ca.jacobian(ca.vec(y), a[j]).reshape((4, 48)) * v[j]
remainder = b - ca.jtimes(momentum, q, v)
weak = ca.Function('arm_weak_terms', [q, v, g, base, world],
                   [ca.densify(momentum), ca.densify(remainder)])
instant = ca.Function('arm_regressor', [q, v, a, g, base, world], [ca.densify(y)])
args.output.mkdir(parents=True, exist_ok=True)
manifest = {'source_sha256': hashlib.sha256(args.regressor.read_bytes()).hexdigest(),
            'parameter_order_per_link': ['mass', 'mx', 'my', 'mz', 'Ixx', 'Iyy', 'Izz',
                                         'Ixy', 'Ixz', 'Iyz', 'viscous', 'coulomb'],
            'joint_order': ['e', 'd', 'c', 'b'], 'casadi_version': ca.__version__}
os.chdir(args.output)
generator = ca.CodeGenerator('arm_regressor.c', {'with_header': True})
generator.add(weak)
generator.add(instant)
generator.generate()
Path('manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
