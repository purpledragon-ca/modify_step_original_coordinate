# modify_original_coordinate

Re-centre and re-orient STEP models before importing them into Isaac Sim.
Auto-detects the bottom-centre face and canonical Z-axis, lets you confirm
or override the choice in a browser-based 3D viewer, and exports a
transformed `.step` plus a JSON transform record.

## Workflow

```
.step file
   │
   ▼
find_bottom_center.py --ui  ──► browser viewer (face picking, preview)
   │
   ▼
*_centered.step  +  transform_record.json
   │
   ▼
Isaac Sim import
```

## Prerequisites

OpenCascade Python bindings (`OCP`), most easily installed via cadquery:

```bash
pip install cadquery
```

## Run

```bash
./start.sh <step_file> [--port N]
```

`start.sh` is a thin wrapper that runs:

```bash
python scripts/find_bottom_center.py <step_file> --ui --port N
```

It launches a local HTTP server (default port `8765`), opens
`http://localhost:8765` in your browser, and prints auto-detected
proposals to stdout. Press **Ctrl+C** to stop.

### Example

```bash
./start.sh my_models/left_shelf.step
./start.sh my_models/left_shelf.step --port 9000
```

### Flags (passed through to the Python script)

| Flag | Default | Purpose |
|---|---|---|
| `--port N` | `8765` | UI/HTTP server port |
| `--no-browser` | off | Don't auto-open a browser tab |
| `--decimals N` | `8` | Cap exported REAL-literal precision |
| `--surfacecurve-mode {0,1,2,3}` | `0` | OCC `write.surfacecurve.mode` |
| `--write-props` | off | Include validation properties (larger files) |

## In the UI

1. Inspect the auto-detected proposals (best one highlighted).
2. Click any face to override the chosen origin / Z-axis.
3. Optionally dial in a Z-rotation to fix X/Y orientation.
4. Click **Export** — the server writes the centered STEP and the
   transform-record JSON to `../processed/<name>_centered.step`
   (alongside a `*_transform.json`).

## HTTP endpoints

The server exposes a small JSON API used by the UI; useful for scripting:

| Method | Path | Purpose |
|---|---|---|
| GET  | `/`                     | Viewer HTML |
| GET  | `/api/model-data`       | Faces, proposals, mesh, bbox of the loaded model |
| GET  | `/api/health`           | Liveness check |
| GET  | `/api/loaded`           | Currently loaded STEP path |
| POST | `/api/load`             | Load a different STEP file without restarting |
| POST | `/api/export-step`      | Apply transform and write `*_centered.step` |
| POST | `/api/transform-record` | Write the transform record JSON |

## Tests

```bash
cd scripts
pytest
```

## Project layout

```
start.sh                              wrapper that launches the UI
scripts/
  find_bottom_center.py               main tool: analysis + UI server
  get_step_component_positions.py     auxiliary: bbox-centre per component
  tests/                              pytest suite
my_models/                            input .step files
isaacsim_models/                      reference / target models
output/                               default export destination
urdf/                                 supporting URDF assets
```
