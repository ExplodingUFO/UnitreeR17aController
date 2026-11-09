"""Independent FK oracle: standard-library 4x4 matrices, no production math code.

The constants below are transcribed from each official URDF joint's origin/axis.
Regenerate fk-reference.json with `python3 resources/models/verify_fk_reference.py`.
Expected matrices are flange/TCP -> base, in row-major order. This oracle checks
model geometry and multiplication order; it is not a hardware calibration.
"""
from math import sin, cos
from pathlib import Path
import json

# (xyz, rpy, signed axis index). Axis index: 1=x, 2=y, 3=z.
CHAINS = {
    "right": [
        ((0, 0, .069), (0, 0, 0), -3),
        ((0, 0, .0469), (0, 0, 0), 1),
        ((0, .1085, -.0047926), (0, 0, 0), 2),
        ((.025, .082408, .016191), (0, 0, 0), -1),
        ((-.022314, -.10786, .062877), (-.017453, 0, 0), 2),
        ((-.000011462, -.1072, .00069991), (0, -.016375, 0), -1),
        ((0, -.01, 0), (-3.1416, 0, -3.1416), -3),
    ],
    "left": [
        ((0, 0, .0689), (0, 0, 0), 3),
        ((0, 0, .047), (0, 0, 0), 1),
        ((0, .1085, -.0047926), (0, 0, 0), -2),
        ((.0350000000000001, .0824079588219821, .0161914700107052), (0, 0, 0), -1),
        ((-.037686, -.10786, .062877), (-.017453, 0, 0), -2),
        ((.0026861, -.1072, .0006561), (0, .016375, 0), -1),
        ((-.002675, -.01, 0), (0, 0, 0), -3),
    ],
}


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rotation(axis, angle):
    # Explicit principal-axis matrices, independent from quaternion recursion.
    c, s = cos(angle), sin(angle)
    if axis == 1:
        return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]
    if axis == 2:
        return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def origin(xyz, rpy):
    result = multiply(multiply(rotation(3, rpy[2]), rotation(2, rpy[1])), rotation(1, rpy[0]))
    for i in range(3):
        result[i][3] = xyz[i]
    return result


def xyzw(rpy):
    r, p, y = [a / 2 for a in rpy]
    return [sin(r)*cos(p)*cos(y)-cos(r)*sin(p)*sin(y),
            cos(r)*sin(p)*cos(y)+sin(r)*cos(p)*sin(y),
            cos(r)*cos(p)*sin(y)-sin(r)*sin(p)*cos(y),
            cos(r)*cos(p)*cos(y)+sin(r)*sin(p)*sin(y)]


def generate():
    result = {"method": "Independent scalar homogeneous matrices; hand-transcribed official joint tables; verify_fk_reference.py", "samples": []}
    for side, chain in CHAINS.items():
        for q, xyz, rpy in [([0.0]*7, (0, 0, 0), (0, 0, 0)),
                            ([.3, .7, 1.1, .9, -.4, .2, -.3], (.03, -.02, .08), (.2, -.3, .4)),
                            ([-.5, 1.5, .2, 2.1, .7, -.3, .8], (-.04, .015, .06), (-.4, .15, -.25))]:
            matrix = identity()
            for angle, (translation, angles, axis) in zip(q, chain):
                matrix = multiply(multiply(matrix, origin(translation, angles)), rotation(abs(axis), angle if axis > 0 else -angle))
            matrix = multiply(matrix, origin(xyz, rpy))
            result["samples"].append({"side": side, "q": q,
                "tcp": {"translation": xyz, "rotation": xyzw(rpy)}, "matrix": matrix})
    return result


if __name__ == "__main__":
    Path(__file__).with_name("fk-reference.json").write_text(json.dumps(generate(), indent=2) + "\n")
