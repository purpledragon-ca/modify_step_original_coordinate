# STEP Asset Finder Web App Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a FastAPI web app that accepts a scene STEP + asset STEP file, finds all asset instances in the scene by name and geometry matching, and displays each instance's XYZ position and Euler angles.

**Architecture:** FastAPI backend with `POST /analyze`; pure Python STEP parser extended to extract transformation matrices and compute XYZ extrinsic Euler angles; plain HTML/JS frontend with client-side JSON/CSV download.

**Tech Stack:** Python 3.10+, FastAPI, uvicorn, python-multipart, pytest, httpx.

---

## File Map

| File | Role |
|------|------|
| `webapp/main.py` | FastAPI app — two endpoints: `GET /` and `POST /analyze` |
| `webapp/step_parser.py` | All STEP parsing: entities, transforms, positions, geometry |
| `webapp/matcher.py` | Name + geometry matching logic |
| `webapp/static/index.html` | Single-page UI — file inputs, table, client-side download |
| `webapp/requirements.txt` | Python dependencies |
| `webapp/tests/test_step_parser.py` | Unit tests for parser functions |
| `webapp/tests/test_matcher.py` | Unit tests for matcher |
| `webapp/tests/test_api.py` | API-level tests |

---

## Task 1: Project skeleton

**Files:**
- Create: `webapp/requirements.txt`
- Create: `webapp/main.py`
- Create: `webapp/static/index.html`
- Create: `webapp/tests/__init__.py`
- Create: `webapp/tests/test_api.py`

- [ ] **Step 1: Create requirements.txt**

`webapp/requirements.txt`:
```
fastapi
uvicorn[standard]
python-multipart
pytest
httpx
pytest-asyncio
```

- [ ] **Step 2: Create stub index.html**

`webapp/static/index.html`:
```html
<!DOCTYPE html>
<html><head><title>STEP Asset Finder</title></head>
<body><h1>STEP Asset Finder</h1><p>Loading...</p></body>
</html>
```

- [ ] **Step 3: Create empty tests/__init__.py**

`webapp/tests/__init__.py`: (empty file)

- [ ] **Step 4: Write failing test for GET /**

`webapp/tests/test_api.py`:
```python
import pytest
from httpx import AsyncClient, ASGITransport
from main import app


@pytest.mark.asyncio
async def test_root_serves_html():
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        r = await client.get("/")
    assert r.status_code == 200
    assert "text/html" in r.headers["content-type"]
```

- [ ] **Step 5: Run test to verify it fails**

```bash
cd webapp && pip install -r requirements.txt
pytest tests/test_api.py::test_root_serves_html -v
```
Expected: `ImportError` — `main` not yet created.

- [ ] **Step 6: Implement main.py**

`webapp/main.py`:
```python
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path

app = FastAPI()

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def root():
    return FileResponse(STATIC_DIR / "index.html")
```

- [ ] **Step 7: Run test to verify it passes**

```bash
cd webapp && pytest tests/test_api.py::test_root_serves_html -v
```
Expected: PASS

- [ ] **Step 8: Commit**

```bash
git add webapp/
git commit -m "feat: project skeleton with FastAPI serving index.html"
```

---

## Task 2: STEP entity parsing utilities

**Files:**
- Create: `webapp/step_parser.py`
- Create: `webapp/tests/test_step_parser.py`

These low-level helpers underpin all subsequent tasks.

- [ ] **Step 1: Write failing tests**

`webapp/tests/test_step_parser.py`:
```python
import math
import pytest
from step_parser import (
    parse_direction, parse_cartesian_point,
    axis2_placement_to_matrix, mat4_multiply, mat4_inverse_rigid,
    rotation_matrix_to_euler_xyz,
)

SAMPLE = {
    '#12': "CARTESIAN_POINT('',(1.,2.,3.))",
    '#13': "DIRECTION('',(0.,0.,1.))",
    '#14': "DIRECTION('',(1.,0.,0.))",
}


def test_parse_direction():
    assert parse_direction(SAMPLE, '#13') == pytest.approx((0.0, 0.0, 1.0))


def test_parse_cartesian_point():
    assert parse_cartesian_point(SAMPLE, '#12') == pytest.approx((1.0, 2.0, 3.0))


def test_axis2_placement_identity():
    M = axis2_placement_to_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert [M[r][0] for r in range(3)] == pytest.approx([1.0, 0.0, 0.0])  # X col
    assert [M[r][1] for r in range(3)] == pytest.approx([0.0, 1.0, 0.0])  # Y col
    assert [M[r][2] for r in range(3)] == pytest.approx([0.0, 0.0, 1.0])  # Z col
    assert [M[r][3] for r in range(3)] == pytest.approx([0.0, 0.0, 0.0])  # origin


def test_rotation_matrix_to_euler_xyz_identity():
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    rx, ry, rz = rotation_matrix_to_euler_xyz(R)
    assert rx == pytest.approx(0.0, abs=1e-9)
    assert ry == pytest.approx(0.0, abs=1e-9)
    assert rz == pytest.approx(0.0, abs=1e-9)


def test_rotation_matrix_to_euler_xyz_z90():
    # Rz(90°): [[0,-1,0],[1,0,0],[0,0,1]]
    R = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    rx, ry, rz = rotation_matrix_to_euler_xyz(R)
    assert rx == pytest.approx(0.0, abs=1e-6)
    assert ry == pytest.approx(0.0, abs=1e-6)
    assert rz == pytest.approx(90.0, abs=1e-4)


def test_mat4_multiply_identity():
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    A = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [0, 0, 0, 1]]
    assert mat4_multiply(A, I) == pytest.approx(A)


def test_mat4_inverse_rigid():
    M = [[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]
    inv = mat4_inverse_rigid(M)
    assert [inv[r][3] for r in range(3)] == pytest.approx([-1.0, -2.0, -3.0])
    product = mat4_multiply(M, inv)
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    assert product == pytest.approx(I)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd webapp && pytest tests/test_step_parser.py -v
```
Expected: `ImportError` — `step_parser` not yet created.

- [ ] **Step 3: Implement step_parser.py (utilities section)**

`webapp/step_parser.py`:
```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd webapp && pytest tests/test_step_parser.py -v
```
Expected: all 7 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add webapp/step_parser.py webapp/tests/test_step_parser.py
git commit -m "feat: STEP parsing utilities with matrix math and XYZ Euler angles"
```

---

## Task 3: Geometry signature extraction

**Files:**
- Modify: `webapp/step_parser.py` (append geometry functions)
- Modify: `webapp/tests/test_step_parser.py` (append geometry tests)

- [ ] **Step 1: Write failing tests**

Append to `webapp/tests/test_step_parser.py`:
```python
from step_parser import collect_cart_points, build_ref_graph, bfs_cart_points, geometry_signature


def _make_box_entities():
    """8 vertices of a 10×20×30 mm box at the origin."""
    pts = [(0,0,0),(10,0,0),(10,20,0),(0,20,0),(0,0,30),(10,0,30),(10,20,30),(0,20,30)]
    return {f'#{100+i}': f"CARTESIAN_POINT('',({x}.,{y}.,{z}.))"
            for i, (x, y, z) in enumerate(pts)}


def test_collect_cart_points():
    pts = collect_cart_points(_make_box_entities())
    assert len(pts) == 8
    assert pts['#100'] == pytest.approx((0.0, 0.0, 0.0))
    assert pts['#107'] == pytest.approx((10.0, 20.0, 30.0))


def test_geometry_signature_box():
    coord_pts = [(0,0,0),(10,0,0),(10,20,0),(0,20,0),(0,0,30),(10,0,30),(10,20,30),(0,20,30)]
    sig = geometry_signature(coord_pts)
    assert sig['volume_mm3'] == pytest.approx(10.0 * 20.0 * 30.0)
    assert sig['diagonal_mm'] == pytest.approx(math.sqrt(10**2 + 20**2 + 30**2))


def test_geometry_signature_single_point():
    sig = geometry_signature([(5.0, 5.0, 5.0)])
    assert sig['volume_mm3'] == pytest.approx(0.0)
    assert sig['diagonal_mm'] == pytest.approx(0.0)


def test_geometry_signature_empty():
    sig = geometry_signature([])
    assert sig['volume_mm3'] == 0.0
    assert sig['diagonal_mm'] == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd webapp && pytest tests/test_step_parser.py::test_collect_cart_points tests/test_step_parser.py::test_geometry_signature_box -v
```
Expected: `ImportError` — functions not yet defined.

- [ ] **Step 3: Append geometry functions to step_parser.py**

```python
# ---------------------------------------------------------------------------
# Geometry signature
# ---------------------------------------------------------------------------

def collect_cart_points(entities: dict) -> dict:
    """Return {eid: (x, y, z)} for every 3-coordinate CARTESIAN_POINT."""
    result = {}
    for eid, body in entities.items():
        if body.startswith('CARTESIAN_POINT'):
            m = re.match(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]+)\)\s*\)", body)
            if m:
                try:
                    coords = tuple(float(x.strip()) for x in m.group(1).split(','))
                    if len(coords) == 3:
                        result[eid] = coords
                except ValueError:
                    pass
    return result


def build_ref_graph(entities: dict, cart_points: dict) -> dict:
    """Forward-reference graph for BFS geometry traversal (skips leaf/unit entities)."""
    ref_pat = re.compile(r'#(\d+)')
    skip = {
        'CARTESIAN_POINT', 'DIRECTION', 'VECTOR', 'UNCERTAINTY_MEASURE',
        'GEOMETRIC_REPRESENTATION_CONTEXT', 'PLANE_ANGLE_MEASURE',
        'DIMENSIONAL_EXPONENTS', 'CONVERSION_BASED_UNIT', 'NAMED_UNIT',
        'LENGTH_UNIT', 'SI_UNIT', 'SOLID_ANGLE_UNIT',
    }
    graph = {}
    for eid, body in entities.items():
        if eid in cart_points:
            continue
        if any(body.startswith(p) for p in skip):
            continue
        refs = ['#' + r for r in ref_pat.findall(body)]
        if refs:
            graph[eid] = refs
    return graph


def bfs_cart_points(start_id: str, ref_graph: dict, cart_points: dict) -> list:
    """BFS from start_id, returning all reachable CARTESIAN_POINT coordinates."""
    visited, queue, pts = set(), [start_id], []
    while queue:
        curr = queue.pop()
        if curr in visited:
            continue
        visited.add(curr)
        if curr in cart_points:
            pts.append(cart_points[curr])
        elif curr in ref_graph:
            queue.extend(r for r in ref_graph[curr] if r not in visited)
    return pts


def geometry_signature(points: list) -> dict:
    """
    Compute bounding-box volume and space diagonal from a list of (x, y, z) tuples.
    Returns {'volume_mm3': float, 'diagonal_mm': float}.
    """
    if not points:
        return {'volume_mm3': 0.0, 'diagonal_mm': 0.0}
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    dx, dy, dz = max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)
    return {
        'volume_mm3': dx * dy * dz,
        'diagonal_mm': math.sqrt(dx * dx + dy * dy + dz * dz),
    }
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd webapp && pytest tests/test_step_parser.py -v
```
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add webapp/step_parser.py webapp/tests/test_step_parser.py
git commit -m "feat: geometry signature extraction (bbox volume + diagonal via BFS)"
```

---

## Task 4: NAUO-to-transform mapping

**Files:**
- Modify: `webapp/step_parser.py` (append transform chain functions)
- Modify: `webapp/tests/test_step_parser.py` (append transform tests)

- [ ] **Step 1: Write failing tests**

Append to `webapp/tests/test_step_parser.py`:
```python
from step_parser import parse_axis2_placement, build_nauo_to_transform

TRANSFORM_ENTITIES = {
    # Identity frame (child/from frame)
    '#10': "CARTESIAN_POINT('',(0.,0.,0.))",
    '#11': "DIRECTION('',(0.,0.,1.))",
    '#12': "DIRECTION('',(1.,0.,0.))",
    '#13': "AXIS2_PLACEMENT_3D('',#10,#11,#12)",
    # Target frame: origin at (100, 200, 50)
    '#20': "CARTESIAN_POINT('',(100.,200.,50.))",
    '#21': "DIRECTION('',(0.,0.,1.))",
    '#22': "DIRECTION('',(1.,0.,0.))",
    '#23': "AXIS2_PLACEMENT_3D('',#20,#21,#22)",
    # IDT: from #13 → to #23
    '#30': "ITEM_DEFINED_TRANSFORMATION('','',#13,#23)",
    # RR_WITH_TRANSFORM referencing IDT #30
    '#35': "(REPRESENTATION_RELATIONSHIP('','',#40,#41)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#30)SHAPE_REPRESENTATION_RELATIONSHIP())",
    # NAUO
    '#50': "NEXT_ASSEMBLY_USAGE_OCCURRENCE('part_1','part','part',#60,#70,$)",
    # PDS pointing to NAUO
    '#51': "PRODUCT_DEFINITION_SHAPE('','',#50)",
    # CDSR: rep=#35, pds=#51
    '#52': "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#35,#51)",
}


def test_parse_axis2_placement_identity():
    M = parse_axis2_placement(TRANSFORM_ENTITIES, '#13')
    assert [M[r][3] for r in range(3)] == pytest.approx([0.0, 0.0, 0.0])
    assert M[0][0] == pytest.approx(1.0)  # x-axis x-component


def test_parse_axis2_placement_translated():
    M = parse_axis2_placement(TRANSFORM_ENTITIES, '#23')
    assert [M[r][3] for r in range(3)] == pytest.approx([100.0, 200.0, 50.0])


def test_build_nauo_to_transform_contains_nauo():
    transforms = build_nauo_to_transform(TRANSFORM_ENTITIES)
    assert '#50' in transforms


def test_build_nauo_to_transform_correct_translation():
    transforms = build_nauo_to_transform(TRANSFORM_ENTITIES)
    T = transforms['#50']
    # T = M_to @ inv(M_from); M_from is identity so T = M_to
    assert [T[r][3] for r in range(3)] == pytest.approx([100.0, 200.0, 50.0])
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd webapp && pytest tests/test_step_parser.py::test_parse_axis2_placement_identity tests/test_step_parser.py::test_build_nauo_to_transform_contains_nauo -v
```
Expected: `ImportError` — functions not yet defined.

- [ ] **Step 3: Append transform chain functions to step_parser.py**

```python
# ---------------------------------------------------------------------------
# Transformation chain extraction
# ---------------------------------------------------------------------------

def _default_x(z_dir: tuple) -> tuple:
    """Choose an X axis perpendicular to z_dir."""
    return (1.0, 0.0, 0.0) if abs(z_dir[0]) < 0.9 else (0.0, 1.0, 0.0)


def parse_axis2_placement(entities: dict, eid: str) -> list:
    """
    Parse AXIS2_PLACEMENT_3D entity → 4×4 homogeneous matrix (local→parent).
    Falls back to identity if entity is missing or malformed.
    """
    val = entities.get(eid, '')
    if not val.startswith('AXIS2_PLACEMENT_3D'):
        return mat4_identity()
    args = parse_args_top(val)
    if len(args) < 3:
        return mat4_identity()
    origin = parse_cartesian_point(entities, args[1])
    z_dir  = parse_direction(entities, args[2])
    x_dir  = parse_direction(entities, args[3]) if len(args) >= 4 else _default_x(z_dir)
    return axis2_placement_to_matrix(origin, z_dir, x_dir)


def build_nauo_to_transform(entities: dict) -> dict:
    """
    Returns {nauo_eid: 4×4_transform} for each NAUO that has an
    associated CONTEXT_DEPENDENT_SHAPE_REPRESENTATION with a transformation.

    Chain: NAUO → PDS → CDSR → RR_WITH_TRANSFORM → IDT → (axis_from, axis_to)
    Transform T = M(axis_to) @ inv(M(axis_from))
    """
    nauo_ids = {eid for eid, v in entities.items()
                if v.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE(')}

    # nauo_id → pds_eid
    nauo_to_pds = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION_SHAPE('):
            args = parse_args_top(val)
            if len(args) >= 3 and args[2] in nauo_ids:
                nauo_to_pds[args[2]] = eid

    # pds_eid → rr_eid (via CDSR)
    pds_to_rr = {}
    for eid, val in entities.items():
        if val.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION('):
            args = parse_args_top(val)
            if len(args) >= 2:
                pds_to_rr[args[1]] = args[0]

    # rr_eid → idt_eid
    rr_to_idt = {}
    for eid, val in entities.items():
        m = re.search(r'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\((#\d+)\)', val)
        if m:
            rr_to_idt[eid] = m.group(1)

    # idt_eid → (axis_from_eid, axis_to_eid)
    idt_to_axes = {}
    for eid, val in entities.items():
        if val.startswith('ITEM_DEFINED_TRANSFORMATION('):
            args = parse_args_top(val)
            if len(args) >= 4:
                idt_to_axes[eid] = (args[2], args[3])

    result = {}
    for nauo_id in nauo_ids:
        pds_id = nauo_to_pds.get(nauo_id)
        if pds_id is None:
            continue
        rr_id = pds_to_rr.get(pds_id)
        if rr_id is None:
            continue
        idt_id = rr_to_idt.get(rr_id)
        if idt_id is None:
            continue
        axes = idt_to_axes.get(idt_id)
        if axes is None:
            continue
        M_from = parse_axis2_placement(entities, axes[0])
        M_to   = parse_axis2_placement(entities, axes[1])
        result[nauo_id] = mat4_multiply(M_to, mat4_inverse_rigid(M_from))

    return result
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd webapp && pytest tests/test_step_parser.py -v
```
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add webapp/step_parser.py webapp/tests/test_step_parser.py
git commit -m "feat: NAUO-to-transform chain extraction from STEP entities"
```

---

## Task 5: Full component extraction (position + rotation + geometry)

**Files:**
- Modify: `webapp/step_parser.py` (append public API functions)
- Modify: `webapp/tests/test_step_parser.py` (append extraction tests)

- [ ] **Step 1: Write failing tests**

Append to `webapp/tests/test_step_parser.py`:
```python
from step_parser import extract_asset_name, extract_components_from_text, extract_asset_geometry

MINIMAL_STEP = """ISO-10303-21;
HEADER;
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF {1 0 10303 442 1 1 4}'));
ENDSEC;
DATA;
#1=PRODUCT('root','root','root',(#2));
#2=PRODUCT_CONTEXT('',#3,'mechanical');
#3=APPLICATION_CONTEXT('');
#4=PRODUCT_DEFINITION_FORMATION('','',#1);
#5=PRODUCT_DEFINITION('design','',#4,#3);
#6=PRODUCT('widget','widget','widget',(#2));
#7=PRODUCT_DEFINITION_FORMATION('','',#6);
#8=PRODUCT_DEFINITION('design','',#7,#3);
#9=NEXT_ASSEMBLY_USAGE_OCCURRENCE('widget_1','widget','widget',#5,#8,$);
ENDSEC;
END-ISO-10303-21;
"""


def test_extract_asset_name():
    assert extract_asset_name(MINIMAL_STEP) == 'root'


def test_extract_components_returns_list():
    components = extract_components_from_text(MINIMAL_STEP)
    assert isinstance(components, list)
    names = [c['name'] for c in components]
    assert 'widget' in names


def test_extract_components_schema():
    components = extract_components_from_text(MINIMAL_STEP)
    c = components[0]
    assert 'prim_path' in c
    assert 'position_mm' in c and set(c['position_mm']) == {'x', 'y', 'z'}
    assert 'euler_deg' in c and set(c['euler_deg']) == {'rx', 'ry', 'rz'}
    assert 'geometry' in c and set(c['geometry']) == {'volume_mm3', 'diagonal_mm'}


def test_extract_asset_name_bad_file():
    with pytest.raises(ValueError):
        extract_asset_name("not a step file")
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd webapp && pytest tests/test_step_parser.py::test_extract_asset_name tests/test_step_parser.py::test_extract_components_returns_list -v
```
Expected: `ImportError` — functions not yet defined.

- [ ] **Step 3: Append public API functions to step_parser.py**

```python
# ---------------------------------------------------------------------------
# Product name and root resolution
# ---------------------------------------------------------------------------

def _build_pd_name_map(entities: dict) -> dict:
    """Map PRODUCT_DEFINITION eid → decoded product name."""
    products = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT('):
            args = parse_args_top(val)
            if args:
                products[eid] = decode_step_string(args[0].strip("'"))

    form_to_prod = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION_FORMATION'):
            refs = re.findall(r'#\d+', val)
            if refs:
                form_to_prod[eid] = refs[-1]

    pd_to_name = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION('):
            for ref in re.findall(r'#\d+', val):
                if ref in form_to_prod:
                    prod_id = form_to_prod[ref]
                    pd_to_name[eid] = products.get(prod_id, f'unknown_{prod_id}')
                    break
    return pd_to_name


def _find_root_pd(entities: dict, pd_to_name: dict) -> str | None:
    """Return the PRODUCT_DEFINITION eid for the root product."""
    for eid, name in pd_to_name.items():
        if name.lower() == 'root':
            return eid
    # Fallback: PD that is never a NAUO child
    all_pds = {eid for eid, v in entities.items() if v.startswith('PRODUCT_DEFINITION(')}
    child_pds = set()
    for eid, val in entities.items():
        if val.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE('):
            args = parse_args_top(val)
            if len(args) >= 5:
                child_pds.add(args[4])
    roots = all_pds - child_pds
    return next(iter(roots)) if roots else None


def _get_nauo_children(parent_pd: str, entities: dict) -> list:
    children = []
    for eid, val in entities.items():
        if val.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE('):
            args = parse_args_top(val)
            if len(args) >= 5 and args[3] == parent_pd:
                children.append({
                    'nauo_id': eid,
                    'name': decode_step_string(args[1].strip("'")),
                    'child_pd': args[4],
                })
    return children


def _build_pd_to_brep_start(entities: dict) -> dict:
    """
    Map PRODUCT_DEFINITION eid → starting entity eid for BFS geometry search.
    Chain: PD → PDS → SDR → SHAPE_REP → SRR → ADVANCED_BREP_SHAPE_REPRESENTATION
    """
    prod_defs = {eid for eid, v in entities.items() if v.startswith('PRODUCT_DEFINITION(')}
    nauo_ids  = {eid for eid, v in entities.items()
                 if v.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE(')}

    pds_to_pd = {}
    for eid, body in entities.items():
        if body.startswith('PRODUCT_DEFINITION_SHAPE('):
            args = parse_args_top(body)
            if len(args) >= 3 and args[2] in prod_defs and args[2] not in nauo_ids:
                pds_to_pd[eid] = args[2]

    sdr = {}
    for eid, body in entities.items():
        if body.startswith('SHAPE_DEFINITION_REPRESENTATION('):
            m = re.match(r'SHAPE_DEFINITION_REPRESENTATION\s*\(\s*(#\d+)\s*,\s*(#\d+)\s*\)', body)
            if m and m.group(1) in pds_to_pd:
                sdr[m.group(1)] = m.group(2)

    adv_breps = {eid for eid, v in entities.items()
                 if v.startswith('ADVANCED_BREP_SHAPE_REPRESENTATION')}
    sr_to_adv = {}
    for eid, body in entities.items():
        if body.startswith('SHAPE_REPRESENTATION_RELATIONSHIP('):
            refs = re.findall(r'#\d+', body)
            if len(refs) >= 2:
                if refs[1] in adv_breps:
                    sr_to_adv[refs[0]] = refs[1]
                elif refs[0] in adv_breps:
                    sr_to_adv[refs[1]] = refs[0]

    pd_to_brep = {}
    for pds_id, pd_id in pds_to_pd.items():
        sr_id = sdr.get(pds_id)
        if sr_id and sr_id in sr_to_adv:
            pd_to_brep[pd_id] = sr_to_adv[sr_id]
    return pd_to_brep


def _walk_assembly(
    parent_pd: str,
    parent_transform: list,
    entities: dict,
    pd_to_name: dict,
    nauo_to_transform: dict,
    cart_points: dict,
    ref_graph: dict,
    pd_to_brep_start: dict,
    parent_path: str,
    visited: set,
) -> list:
    results = []
    for child in _get_nauo_children(parent_pd, entities):
        child_pd = child['child_pd']
        if child_pd in visited:
            continue

        raw_name  = pd_to_name.get(child_pd, child['name'] or f'prim_{child_pd[1:]}')
        safe_name = re.sub(r'[/ ]+', '_', raw_name).strip('_') or f'prim_{child_pd[1:]}'
        prim_path = f"{parent_path}/{safe_name}"

        local_T  = nauo_to_transform.get(child['nauo_id'], mat4_identity())
        world_T  = mat4_multiply(parent_transform, local_T)

        pos_mm = {'x': world_T[0][3], 'y': world_T[1][3], 'z': world_T[2][3]}
        R = [[world_T[i][j] for j in range(3)] for i in range(3)]
        rx, ry, rz = rotation_matrix_to_euler_xyz(R)
        euler_deg = {'rx': rx, 'ry': ry, 'rz': rz}

        brep_start = pd_to_brep_start.get(child_pd)
        pts = bfs_cart_points(brep_start, ref_graph, cart_points) if brep_start else []
        geo = geometry_signature(pts)

        results.append({
            'prim_path': prim_path,
            'name': raw_name,
            'position_mm': pos_mm,
            'euler_deg': euler_deg,
            'geometry': geo,
        })
        results.extend(_walk_assembly(
            child_pd, world_T, entities, pd_to_name,
            nauo_to_transform, cart_points, ref_graph, pd_to_brep_start,
            prim_path, visited | {child_pd},
        ))
    return results


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def extract_asset_name(step_text: str) -> str:
    """Return the root product name from a STEP file."""
    entities = parse_step_entities(step_text)
    pd_to_name = _build_pd_name_map(entities)
    root_pd = _find_root_pd(entities, pd_to_name)
    if root_pd is None:
        raise ValueError("Could not determine root assembly")
    return pd_to_name.get(root_pd, 'unknown')


def extract_components_from_text(step_text: str) -> list:
    """
    Parse a scene STEP file and return all assembly components with:
    prim_path, name, position_mm {x,y,z}, euler_deg {rx,ry,rz}, geometry {volume_mm3,diagonal_mm}
    """
    entities = parse_step_entities(step_text)
    pd_to_name        = _build_pd_name_map(entities)
    root_pd           = _find_root_pd(entities, pd_to_name)
    if root_pd is None:
        raise ValueError("Could not determine root assembly in scene STEP")

    nauo_to_transform = build_nauo_to_transform(entities)
    cart_points       = collect_cart_points(entities)
    ref_graph         = build_ref_graph(entities, cart_points)
    pd_to_brep        = _build_pd_to_brep_start(entities)

    root_name  = pd_to_name.get(root_pd, 'Root')
    safe_root  = re.sub(r'[/ ]+', '_', root_name).strip('_') or 'Root'

    return _walk_assembly(
        parent_pd=root_pd,
        parent_transform=mat4_identity(),
        entities=entities,
        pd_to_name=pd_to_name,
        nauo_to_transform=nauo_to_transform,
        cart_points=cart_points,
        ref_graph=ref_graph,
        pd_to_brep_start=pd_to_brep,
        parent_path=f"/{safe_root}",
        visited={root_pd},
    )


def extract_asset_geometry(step_text: str) -> dict:
    """
    Parse an asset STEP file and return its geometry signature using all
    CARTESIAN_POINTs in the file (the asset IS the entire file).
    """
    entities = parse_step_entities(step_text)
    cart_points = collect_cart_points(entities)
    return geometry_signature(list(cart_points.values()))
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd webapp && pytest tests/test_step_parser.py -v
```
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add webapp/step_parser.py webapp/tests/test_step_parser.py
git commit -m "feat: full component extraction with world position, Euler angles, and geometry"
```

---

## Task 6: matcher.py — name and geometry matching

**Files:**
- Create: `webapp/matcher.py`
- Create: `webapp/tests/test_matcher.py`

- [ ] **Step 1: Write failing tests**

`webapp/tests/test_matcher.py`:
```python
import pytest
from matcher import match_components

SCENE_COMPONENTS = [
    {
        'prim_path': '/Root/shelf/cc_waste_bin_1',
        'name': 'cc_waste_bin',
        'position_mm': {'x': 100.0, 'y': 200.0, 'z': 50.0},
        'euler_deg': {'rx': 0.0, 'ry': 0.0, 'rz': 90.0},
        'geometry': {'volume_mm3': 10000.0, 'diagonal_mm': 200.0},
    },
    {
        'prim_path': '/Root/shelf/other_part',
        'name': 'other_part',
        'position_mm': {'x': 300.0, 'y': 200.0, 'z': 50.0},
        'euler_deg': {'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
        'geometry': {'volume_mm3': 50000.0, 'diagonal_mm': 500.0},
    },
    {
        'prim_path': '/Root/shelf/cc_waste_bin_2',
        'name': 'different_name',
        'position_mm': {'x': 500.0, 'y': 200.0, 'z': 50.0},
        'euler_deg': {'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
        'geometry': {'volume_mm3': 10000.0, 'diagonal_mm': 200.0},
    },
]
ASSET_NAME = 'cc_waste_bin'
ASSET_GEO  = {'volume_mm3': 10000.0, 'diagonal_mm': 200.0}


def test_name_match():
    matches = match_components(SCENE_COMPONENTS, ASSET_NAME, ASSET_GEO)
    paths = [m['prim_path'] for m in matches]
    assert '/Root/shelf/cc_waste_bin_1' in paths


def test_geometry_match():
    matches = match_components(SCENE_COMPONENTS, ASSET_NAME, ASSET_GEO)
    paths = [m['prim_path'] for m in matches]
    assert '/Root/shelf/cc_waste_bin_2' in paths


def test_no_match_for_unrelated():
    matches = match_components(SCENE_COMPONENTS, ASSET_NAME, ASSET_GEO)
    paths = [m['prim_path'] for m in matches]
    assert '/Root/shelf/other_part' not in paths


def test_match_type_name_and_geometry():
    matches = match_components(SCENE_COMPONENTS, ASSET_NAME, ASSET_GEO)
    bin1 = next(m for m in matches if m['prim_path'] == '/Root/shelf/cc_waste_bin_1')
    assert bin1['match_type'] == 'name+geometry'


def test_match_type_geometry_only():
    matches = match_components(SCENE_COMPONENTS, ASSET_NAME, ASSET_GEO)
    bin2 = next(m for m in matches if m['prim_path'] == '/Root/shelf/cc_waste_bin_2')
    assert bin2['match_type'] == 'geometry'


def test_zero_volume_asset_skips_geometry_match():
    """An asset with no geometry must not trigger false geometry matches."""
    matches = match_components(SCENE_COMPONENTS, 'nonexistent', {'volume_mm3': 0.0, 'diagonal_mm': 0.0})
    assert matches == []
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd webapp && pytest tests/test_matcher.py -v
```
Expected: `ImportError` — `matcher` not yet created.

- [ ] **Step 3: Implement matcher.py**

`webapp/matcher.py`:
```python
GEO_TOLERANCE = 0.95  # both ratios must be >= this for a geometry match


def _geo_ratio(a: float, b: float) -> float:
    if max(a, b) < 1e-9:
        return 1.0
    return min(a, b) / max(a, b)


def _geo_match(asset_geo: dict, comp_geo: dict) -> bool:
    """True if both volume and diagonal are within GEO_TOLERANCE of each other."""
    av, cv = asset_geo['volume_mm3'], comp_geo['volume_mm3']
    ad, cd = asset_geo['diagonal_mm'], comp_geo['diagonal_mm']
    if av <= 0 or ad <= 0:
        return False
    return _geo_ratio(av, cv) >= GEO_TOLERANCE and _geo_ratio(ad, cd) >= GEO_TOLERANCE


def match_components(
    scene_components: list,
    asset_name: str,
    asset_geometry: dict,
) -> list:
    """
    Find scene components matching the asset by name and/or geometry.
    Returns component dicts augmented with 'match_type':
      'name', 'geometry', or 'name+geometry'.
    """
    asset_name_lower = asset_name.lower()
    results = []
    for comp in scene_components:
        name_hit = asset_name_lower in comp['name'].lower()
        geo_hit  = _geo_match(asset_geometry, comp['geometry'])
        if not name_hit and not geo_hit:
            continue
        match_type = '+'.join(t for t in ['name' if name_hit else '', 'geometry' if geo_hit else ''] if t)
        results.append({**comp, 'match_type': match_type})
    return results
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd webapp && pytest tests/test_matcher.py -v
```
Expected: all 6 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add webapp/matcher.py webapp/tests/test_matcher.py
git commit -m "feat: name+geometry asset matcher with 5% tolerance"
```

---

## Task 7: POST /analyze endpoint

**Files:**
- Modify: `webapp/main.py`
- Modify: `webapp/tests/test_api.py`

- [ ] **Step 1: Write failing tests**

Append to `webapp/tests/test_api.py`:
```python
import io
from pathlib import Path

SCENE_STEP = Path(__file__).parent.parent.parent / "my_models" / "front_shelf.step"
ASSET_STEP  = Path(__file__).parent.parent.parent / "my_models" / "cc_waste_bin.step"


@pytest.mark.asyncio
async def test_analyze_returns_json():
    if not SCENE_STEP.exists() or not ASSET_STEP.exists():
        pytest.skip("Real STEP files not present")
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        r = await client.post(
            "/analyze",
            files={
                "scene_file": ("front_shelf.step", SCENE_STEP.read_bytes(), "application/octet-stream"),
                "asset_file":  ("cc_waste_bin.step", ASSET_STEP.read_bytes(), "application/octet-stream"),
            },
        )
    assert r.status_code == 200
    data = r.json()
    assert "matches" in data
    assert "total" in data
    assert "asset_geometry" in data


@pytest.mark.asyncio
async def test_analyze_bad_file_returns_400():
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        r = await client.post(
            "/analyze",
            files={
                "scene_file": ("bad.step",  b"not a step file", "application/octet-stream"),
                "asset_file":  ("bad2.step", b"also bad",        "application/octet-stream"),
            },
        )
    assert r.status_code == 400
    assert "error" in r.json()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd webapp && pytest tests/test_api.py -v
```
Expected: `test_analyze_bad_file_returns_400` fails (no `/analyze` yet), real-file test skips.

- [ ] **Step 3: Replace main.py with full implementation**

`webapp/main.py`:
```python
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path

from step_parser import (
    extract_asset_geometry,
    extract_asset_name,
    extract_components_from_text,
)
from matcher import match_components

app = FastAPI()

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def root():
    return FileResponse(STATIC_DIR / "index.html")


@app.post("/analyze")
async def analyze(
    scene_file: UploadFile = File(...),
    asset_file:  UploadFile = File(...),
):
    scene_bytes = await scene_file.read()
    asset_bytes  = await asset_file.read()

    scene_text = scene_bytes.decode("utf-8", errors="replace")
    asset_text  = asset_bytes.decode("utf-8", errors="replace")

    try:
        scene_components = extract_components_from_text(scene_text)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail={"error": str(exc)})

    try:
        asset_name = extract_asset_name(asset_text)
        asset_geo  = extract_asset_geometry(asset_text)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail={"error": str(exc)})

    matches = match_components(scene_components, asset_name, asset_geo)

    return {
        "scene_file":     scene_file.filename,
        "asset_file":      asset_file.filename,
        "asset_geometry": asset_geo,
        "matches":        matches,
        "total":          len(matches),
    }
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd webapp && pytest tests/test_api.py -v
```
Expected: `test_root_serves_html` PASS, `test_analyze_bad_file_returns_400` PASS, `test_analyze_returns_json` PASS or SKIP.

- [ ] **Step 5: Commit**

```bash
git add webapp/main.py webapp/tests/test_api.py
git commit -m "feat: POST /analyze endpoint wiring parser and matcher"
```

---

## Task 8: index.html — complete UI

**Files:**
- Modify: `webapp/static/index.html`

- [ ] **Step 1: Replace stub with full UI**

`webapp/static/index.html`:
```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>STEP Asset Finder</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: system-ui, sans-serif; background: #f5f5f5; color: #222; padding: 2rem; }
    h1 { font-size: 1.5rem; margin-bottom: 1.5rem; }

    .form-card {
      background: #fff; border-radius: 8px; padding: 1.5rem;
      box-shadow: 0 1px 4px rgba(0,0,0,.12); max-width: 600px; margin-bottom: 1.5rem;
    }
    .field { margin-bottom: 1rem; }
    label { display: block; font-weight: 600; margin-bottom: .4rem; font-size: .9rem; }
    input[type=file] { width: 100%; }

    button {
      background: #2563eb; color: #fff; border: none; border-radius: 6px;
      padding: .6rem 1.4rem; font-size: 1rem; cursor: pointer;
    }
    button:disabled { background: #93c5fd; cursor: not-allowed; }

    .spinner {
      display: inline-block; width: 1rem; height: 1rem;
      border: 2px solid #fff; border-top-color: transparent;
      border-radius: 50%; animation: spin .7s linear infinite; margin-right: .5rem;
    }
    @keyframes spin { to { transform: rotate(360deg); } }

    #error {
      background: #fee2e2; color: #b91c1c; border-radius: 6px;
      padding: .8rem 1rem; margin-bottom: 1rem; display: none;
    }

    #results { display: none; }
    .results-header {
      display: flex; align-items: center; gap: 1rem;
      margin-bottom: 1rem; flex-wrap: wrap;
    }
    .results-header h2 { font-size: 1.1rem; }
    .summary { color: #555; font-size: .9rem; }
    .download-row { display: flex; gap: .5rem; margin-left: auto; }
    .dl-btn { background: #16a34a; }

    table {
      width: 100%; border-collapse: collapse; background: #fff;
      border-radius: 8px; overflow: hidden; box-shadow: 0 1px 4px rgba(0,0,0,.12);
    }
    th { background: #f0f0f0; text-align: left; padding: .6rem .8rem; font-size: .85rem; }
    td { padding: .55rem .8rem; border-top: 1px solid #e5e5e5; font-size: .85rem; font-family: monospace; }
    tr:hover td { background: #fafafa; }

    .badge {
      display: inline-block; padding: .15rem .5rem;
      border-radius: 999px; font-size: .75rem; font-family: system-ui; font-weight: 600;
    }
    .badge-name-geometry { background: #dbeafe; color: #1d4ed8; }
    .badge-name          { background: #dcfce7; color: #15803d; }
    .badge-geometry      { background: #fef9c3; color: #a16207; }
  </style>
</head>
<body>
  <h1>STEP Asset Finder</h1>

  <div class="form-card">
    <div class="field">
      <label for="sceneFile">Scene STEP file</label>
      <input type="file" id="sceneFile" accept=".step,.stp">
    </div>
    <div class="field">
      <label for="assetFile">Asset STEP file</label>
      <input type="file" id="assetFile" accept=".step,.stp">
    </div>
    <button id="analyzeBtn" disabled>Analyze</button>
  </div>

  <div id="error"></div>

  <div id="results">
    <div class="results-header">
      <h2 id="resultsTitle"></h2>
      <span class="summary" id="resultsSummary"></span>
      <div class="download-row">
        <button class="dl-btn" id="dlJson">Download JSON</button>
        <button class="dl-btn" id="dlCsv">Download CSV</button>
      </div>
    </div>
    <table>
      <thead>
        <tr>
          <th>Prim Path</th>
          <th>Match</th>
          <th>X (mm)</th><th>Y (mm)</th><th>Z (mm)</th>
          <th>Rx (°)</th><th>Ry (°)</th><th>Rz (°)</th>
        </tr>
      </thead>
      <tbody id="tableBody"></tbody>
    </table>
  </div>

  <script>
    const sceneInput = document.getElementById('sceneFile');
    const assetInput = document.getElementById('assetFile');
    const btn        = document.getElementById('analyzeBtn');
    const errorEl    = document.getElementById('error');
    const resultsEl  = document.getElementById('results');
    let lastResult   = null;

    function checkReady() {
      btn.disabled = !(sceneInput.files.length && assetInput.files.length);
    }
    sceneInput.addEventListener('change', checkReady);
    assetInput.addEventListener('change', checkReady);

    btn.addEventListener('click', async () => {
      errorEl.style.display = 'none';
      resultsEl.style.display = 'none';
      btn.innerHTML = '<span class="spinner"></span>Analyzing…';
      btn.disabled = true;

      const fd = new FormData();
      fd.append('scene_file', sceneInput.files[0]);
      fd.append('asset_file',  assetInput.files[0]);

      try {
        const resp = await fetch('/analyze', { method: 'POST', body: fd });
        const data = await resp.json();
        if (!resp.ok) throw new Error(data.detail?.error || data.detail || 'Server error');
        lastResult = data;
        renderResults(data);
      } catch (err) {
        errorEl.textContent = err.message;
        errorEl.style.display = 'block';
      } finally {
        btn.innerHTML = 'Analyze';
        btn.disabled = false;
      }
    });

    function fmt(v) { return v == null ? '—' : Number(v).toFixed(3); }

    function badgeClass(t) {
      if (t === 'name+geometry') return 'badge-name-geometry';
      if (t === 'name') return 'badge-name';
      return 'badge-geometry';
    }

    function renderResults(data) {
      document.getElementById('resultsTitle').textContent =
        `Found ${data.total} instance(s) of "${data.asset_file}"`;
      const geo = data.asset_geometry;
      document.getElementById('resultsSummary').textContent =
        `Asset: ${geo.volume_mm3.toFixed(1)} mm³ · ${geo.diagonal_mm.toFixed(1)} mm diagonal`;

      const tbody = document.getElementById('tableBody');
      tbody.innerHTML = '';
      for (const m of data.matches) {
        const p = m.position_mm, e = m.euler_deg;
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td>${m.prim_path}</td>
          <td><span class="badge ${badgeClass(m.match_type)}">${m.match_type}</span></td>
          <td>${fmt(p.x)}</td><td>${fmt(p.y)}</td><td>${fmt(p.z)}</td>
          <td>${fmt(e.rx)}</td><td>${fmt(e.ry)}</td><td>${fmt(e.rz)}</td>`;
        tbody.appendChild(tr);
      }
      resultsEl.style.display = 'block';
    }

    function triggerDownload(blob, filename) {
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url; a.download = filename;
      document.body.appendChild(a); a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    }

    document.getElementById('dlJson').addEventListener('click', () => {
      if (!lastResult) return;
      triggerDownload(
        new Blob([JSON.stringify(lastResult, null, 2)], { type: 'application/json' }),
        'step_asset_matches.json'
      );
    });

    document.getElementById('dlCsv').addEventListener('click', () => {
      if (!lastResult) return;
      const header = 'prim_path,name,match_type,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg\n';
      const rows = lastResult.matches.map(m => {
        const p = m.position_mm, e = m.euler_deg;
        return [m.prim_path, m.name, m.match_type,
                p.x, p.y, p.z, e.rx, e.ry, e.rz].join(',');
      }).join('\n');
      triggerDownload(
        new Blob([header + rows], { type: 'text/csv' }),
        'step_asset_matches.csv'
      );
    });
  </script>
</body>
</html>
```

- [ ] **Step 2: Start server and visually verify the UI**

```bash
cd webapp && uvicorn main:app --reload
# Open http://localhost:8000
```
Check: two file inputs visible, Analyze button disabled, no console errors.

- [ ] **Step 3: Commit**

```bash
git add webapp/static/index.html
git commit -m "feat: complete single-page UI with upload, results table, and client-side download"
```

---

## Task 9: End-to-end verification

**Files:** no code changes — validation only.

- [ ] **Step 1: Run full test suite**

```bash
cd webapp && pytest tests/ -v
```
Expected: all tests PASS (real-file test PASS if STEP files are present, SKIP otherwise).

- [ ] **Step 2: Manual test with real STEP files**

```bash
cd webapp && uvicorn main:app --reload
```
Open http://localhost:8000. Upload:
- Scene: `my_models/front_shelf.step`
- Asset: `my_models/cc_waste_bin.step`

Click Analyze. Verify:
- Results table appears with ≥ 1 row
- Rows show non-zero X/Y/Z values
- Euler angle columns appear (may be 0° for identity rotations)
- Match type badges render correctly

- [ ] **Step 3: Verify CSV download**

Click "Download CSV". Open the file and confirm columns:
```
prim_path,name,match_type,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg
```

- [ ] **Step 4: Verify JSON download**

Click "Download JSON". Open the file and confirm it matches the response schema from the spec.

- [ ] **Step 5: Final commit**

```bash
git add .
git commit -m "chore: end-to-end verification complete"
```
