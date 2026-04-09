import re
import math


# ---------------------------------------------------------------------------
# Text utilities
# ---------------------------------------------------------------------------

def decode_step_string(s: str) -> str:
    def replace(m):
        hex_chars = m.group(1)
        try:
            return ''.join(chr(int(hex_chars[i:i + 4], 16)) for i in range(0, len(hex_chars), 4))
        except Exception:
            return m.group(0)
    return re.sub(r'\\X2\\([0-9A-Fa-f]+)\\X0\\', replace, s)


def parse_step_entities(text: str) -> dict:
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    m = re.search(r'\bDATA\s*;(.*?)\bENDSEC\b', text, re.DOTALL | re.IGNORECASE)
    if not m:
        raise ValueError("No DATA section found in STEP file")
    entities = {}
    for stmt in m.group(1).split(';'):
        stmt = stmt.strip()
        if not stmt:
            continue
        m2 = re.match(r'(#\d+)\s*=\s*(.*)', stmt, re.DOTALL)
        if m2:
            entities[m2.group(1)] = ' '.join(m2.group(2).split())
    return entities


def parse_args_top(val: str) -> list:
    m = re.match(r'[A-Z_0-9]*\((.*)\)\s*$', val, re.DOTALL)
    if not m:
        m = re.match(r'\((.*)\)\s*$', val, re.DOTALL)
        if not m:
            return []
    inner = m.group(1)
    args, depth, current = [], 0, []
    for ch in inner:
        if ch == '(':
            depth += 1
            current.append(ch)
        elif ch == ')':
            depth -= 1
            current.append(ch)
        elif ch == ',' and depth == 0:
            args.append(''.join(current).strip())
            current = []
        else:
            current.append(ch)
    if current:
        args.append(''.join(current).strip())
    return args


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

def parse_direction(entities: dict, eid: str) -> tuple:
    """Return (dx, dy, dz) from a DIRECTION entity."""
    val = entities.get(eid, '')
    m = re.match(r"DIRECTION\s*\(\s*'[^']*'\s*,\s*\(([^)]+)\)\s*\)", val)
    if not m:
        return (0.0, 0.0, 1.0)
    return tuple(float(x.strip()) for x in m.group(1).split(','))


def parse_cartesian_point(entities: dict, eid: str) -> tuple:
    """Return (x, y, z) from a CARTESIAN_POINT entity."""
    val = entities.get(eid, '')
    m = re.match(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]+)\)\s*\)", val)
    if not m:
        return (0.0, 0.0, 0.0)
    return tuple(float(x.strip()) for x in m.group(1).split(','))


# ---------------------------------------------------------------------------
# 4×4 matrix math (pure Python, row-major: M[row][col])
# ---------------------------------------------------------------------------

def mat4_identity() -> list:
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def mat4_multiply(A: list, B: list) -> list:
    result = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            result[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
    return result


def mat4_inverse_rigid(M: list) -> list:
    """Inverse of a rigid-body (rotation + translation only) 4×4 matrix."""
    Rt = [[M[j][i] for j in range(3)] for i in range(3)]
    t = [M[i][3] for i in range(3)]
    neg_Rt_t = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [
        [Rt[0][0], Rt[0][1], Rt[0][2], neg_Rt_t[0]],
        [Rt[1][0], Rt[1][1], Rt[1][2], neg_Rt_t[1]],
        [Rt[2][0], Rt[2][1], Rt[2][2], neg_Rt_t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _normalize(v: tuple) -> tuple:
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-12:
        return v
    return tuple(x / n for x in v)


def _cross(a: tuple, b: tuple) -> tuple:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def axis2_placement_to_matrix(origin: tuple, z_dir: tuple, x_dir: tuple) -> list:
    """
    Build a 4×4 homogeneous matrix from AXIS2_PLACEMENT_3D components.
    The matrix transforms points from the local frame to the parent frame.
    Columns: [x_axis | y_axis | z_axis | origin].
    """
    z = _normalize(z_dir)
    x_raw = _normalize(x_dir)
    dot_zx = sum(z[i] * x_raw[i] for i in range(3))
    x = _normalize(tuple(x_raw[i] - dot_zx * z[i] for i in range(3)))
    y = _cross(z, x)
    return [
        [x[0], y[0], z[0], origin[0]],
        [x[1], y[1], z[1], origin[1]],
        [x[2], y[2], z[2], origin[2]],
        [0.0,  0.0,  0.0,  1.0],
    ]


def rotation_matrix_to_euler_xyz(R: list) -> tuple:
    """
    Convert a 3×3 rotation matrix to XYZ extrinsic Euler angles in degrees.
    R = Rz(rz) @ Ry(ry) @ Rx(rx)  (extrinsic XYZ = intrinsic ZYX)
    Returns (rx_deg, ry_deg, rz_deg).
    """
    ry = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(ry)) > 1e-6:
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    else:
        rx = 0.0
        rz = math.atan2(-R[0][1], R[1][1])
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)
