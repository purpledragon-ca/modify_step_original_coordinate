# usd_step_file_modify

Webapp for repairing STEP files exported from Shapr3D before importing into Isaac Sim.

## Workflow overview

```
Shapr3D export (.step)
        │
        ▼
Repair — fix syntax errors and Z-axis orientation
        │
        ▼
Isaac Sim import
```

---

## STEP Repair

The webapp fixes two defects that Shapr3D reliably produces in every export:

| Defect | Symptom | Fix |
|--------|---------|-----|
| Bare-integer REAL literals (`-0`, `1`) in `DIRECTION` vectors | OpenCASCADE parser reports *"Incorrect Syntax : Fails Count : N"* | Convert to valid STEP notation (`0.`, `1.`) |
| Global product frame set to Z-down `(0,0,−1)` | Components load with wrong rotation in Isaac Sim / Shapr3D re-import | Flip all assembly-level placement frames to Z-up |

### Start the server

```bash
cd webapp
pip install -r requirements.txt
uvicorn main:app --reload --port 8000
```

Then open **http://localhost:8000** in a browser.

### Use the UI

1. Drag your `.step` file onto the drop zone (or click **browse**).
2. Leave both checkboxes ticked (they fix the two defects described above).
3. Click **Repair & Download**.
4. The browser downloads `<original_name>_repaired.step`.  
   The result box shows how many lines were fixed and how many Z-directions were flipped.

### Use the API directly

```bash
curl -X POST http://localhost:8000/repair \
  -F "file=@left_shelf.step" \
  -F "fix_syntax=true" \
  -F "fix_z=true" \
  -o left_shelf_repaired.step
```

Response headers report what changed:

```
X-Repair-Summary:        54377 line(s) had bare-integer REAL literals fixed; 200 assembly Z-direction(s) flipped to Z-up
X-Syntax-Lines-Fixed:    54377
X-Z-Directions-Flipped:  200
```

### Run tests

```bash
cd webapp
pytest
```

---

## Project layout

```
webapp/                 FastAPI app — STEP repair UI and API
  main.py               Routes: GET / (UI), POST /repair
  step_repair.py        Repair logic (syntax fix + Z-axis flip)
  step_parser.py        STEP entity parser, matrix math
  static/index.html     Browser UI
  tests/                pytest suite
```
