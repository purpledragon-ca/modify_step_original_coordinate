# STEP Asset Finder Web App — Design Spec

**Date:** 2026-04-09  
**Status:** Approved

---

## Overview

A Python-based web application that accepts a scene STEP file and an asset STEP file, finds all instances of the asset within the scene, and reports each instance's relative pose (XYZ position + XYZ Euler angles).

---

## Architecture

### File Structure

```
usd_step_file_modify/
└── webapp/
    ├── main.py              # FastAPI app and endpoints
    ├── step_parser.py       # Extended STEP parser (positions + rotations)
    ├── matcher.py           # Name + geometry matching logic
    └── static/
        └── index.html       # Single-page UI (no framework)
```

### Runtime Flow

1. User opens browser → `index.html` served by FastAPI at `GET /`
2. User uploads **scene STEP** + **asset STEP** via form
3. `POST /analyze` → parser reads both files, matcher finds instances, returns JSON
4. Frontend renders results table with position and Euler angles per match
5. User clicks Download JSON or Download CSV → frontend generates file client-side from the result JSON already in memory (Blob URL, no extra server round-trip)

### Tech Stack

- **Backend:** Python, FastAPI, uvicorn
- **Frontend:** Plain HTML/CSS/JS, no build step
- **STEP parsing:** Extended from existing `scripts/extract_prims_and_positions.py`

---

## Components

### `step_parser.py`

Extends existing STEP parsing logic with two new capabilities:

**Rotation extraction:**
- Parse `AXIS2_PLACEMENT_3D` entities: each defines a local coordinate frame via origin point, Z-axis direction, and X-axis direction. Y-axis is derived as Z × X.
- Parse `ITEM_DEFINED_TRANSFORMATION` entities: link two `AXIS2_PLACEMENT_3D` instances (from-frame → to-frame), defining the placement of a child component relative to its parent.
- Walk the assembly tree via `NEXT_ASSEMBLY_USAGE_OCCURRENCE` → `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` → `ITEM_DEFINED_TRANSFORMATION`, accumulating 4×4 homogeneous transformation matrices from root to each leaf.
- Convert the 3×3 rotation sub-matrix of the accumulated transform to XYZ extrinsic Euler angles (in degrees).

**Geometry signature extraction:**
- Collect all `CARTESIAN_POINT` coordinates reachable from a component's B-rep via BFS (same BFS used for bbox center).
- Compute: bounding box volume (W × H × D in mm³) and space diagonal length (mm).
- Return these as the component's geometry signature.

**Output per component:**
```python
{
    "prim_path": str,
    "name": str,
    "position_mm": {"x": float, "y": float, "z": float},
    "euler_deg": {"rx": float, "ry": float, "rz": float},
    "geometry": {"volume_mm3": float, "diagonal_mm": float},
}
```

### `matcher.py`

Matches asset to scene components using two independent criteria:

- **Name match:** scene component name contains or equals the asset's root product name (case-insensitive).
- **Geometry match:** both volume ratio and diagonal ratio between asset and scene component are within 5% tolerance.
  - `ratio = min(a, b) / max(a, b) >= 0.95`

A component is included in results if it satisfies name match OR geometry match (or both). The `match_type` field records which criteria were satisfied: `"name"`, `"geometry"`, or `"name+geometry"`.

### `main.py`

**Endpoints:**

```
POST /analyze
  Body: multipart/form-data
    scene_file: UploadFile (.step)
    asset_file:  UploadFile (.step)
  Response: JSON

GET /
  Response: serves static/index.html
```

Downloads are handled client-side: the frontend converts the result JSON to a Blob and triggers a browser download. No server endpoint needed.

**Response JSON schema:**
```json
{
  "scene_file": "front_shelf.step",
  "asset_file": "cc_waste_bin.step",
  "asset_geometry": {
    "volume_mm3": 12345.6,
    "diagonal_mm": 234.5
  },
  "matches": [
    {
      "prim_path": "/Root/front_shelf/cc_waste_bin_1",
      "name": "cc_waste_bin",
      "match_type": "name+geometry",
      "position_mm": { "x": 100.0, "y": 200.0, "z": 50.0 },
      "euler_deg": { "rx": 0.0, "ry": 0.0, "rz": 90.0 }
    }
  ],
  "total": 3
}
```

### `static/index.html`

Single-file UI with:
- Two file inputs: scene STEP and asset STEP
- Analyze button (disabled until both files selected)
- Loading spinner during `POST /analyze`
- Error banner on failure
- Results table: prim path, match type badge, X/Y/Z position (mm), Rx/Ry/Rz Euler angles (degrees)
- Download JSON and Download CSV buttons (appear after results load)

---

## Euler Angle Convention

**XYZ extrinsic** (rotate around world X first, then world Y, then world Z). Reported in degrees. This is equivalent to ZYX intrinsic (body-fixed) rotation.

---

## Geometry Matching Tolerance

Both volume ratio and bounding box diagonal ratio must be ≥ 0.95 (within 5%) for a geometry match. Tolerance is a constant in `matcher.py` and can be adjusted.

---

## Error Handling

- Invalid or non-STEP file uploaded → 400 response with descriptive message
- No root assembly found in scene → 422 response
- Asset file has no geometry → 422 response with explanation
- All errors surfaced as JSON `{"error": "..."}` and displayed in the UI error banner

---

## Dependencies

```
fastapi
uvicorn
python-multipart
```

No heavy dependencies (no numpy, no OpenCASCADE). All STEP parsing is pure Python regex-based, consistent with existing scripts.

---

## Running the App

```bash
cd webapp
pip install fastapi uvicorn python-multipart
uvicorn main:app --reload
# Open http://localhost:8000
```
