# usd_step_file_modify

Tools for cleaning, restructuring, and translating STEP files exported from Shapr3D before importing into Isaac Sim.

## Workflow overview

```
Shapr3D export (.step)
        │
        ▼
1. Repair   — fix syntax errors and Z-axis orientation   (webapp or CLI)
        │
        ▼
2. Translate — Chinese part names → English              (scripts/)
        │
        ▼
3. Restructure — reorganise assembly hierarchy           (scripts/)
        │
        ▼
Isaac Sim import
```

---

## 1. STEP Repair — webapp

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

## 2. Translate Chinese part names

Shapr3D encodes Chinese text as `\X2\<hex>\X0\` inside STEP strings.  
The translation script replaces them with English using a persistent lookup table.

```bash
python scripts/translate_step_cn_to_en.py my_models/front_shelf.step
```

Produces `my_models/front_shelf_en.step`.  
Unknown strings are appended as placeholders in `config/cn_en_translations.json` for you to fill in.

**First run** (build the translation table from a CSV comparison file):

```bash
# place a <stem>_cn_en_comparison.csv next to the .step file, then:
python scripts/translate_step_cn_to_en.py my_models/front_shelf.step
```

---

## 3. Inspect component names

Before restructuring you need the exact part names as they appear in the STEP file:

```bash
python scripts/0_get_step_component_names.py my_models/front_shelf_en.step
```

Writes `component_names.txt` (one name per line).

---

## 4. Restructure the assembly hierarchy

Reorganises the assembly tree into a clean three-level structure (`root → group → part`) using a markdown config file.

**Write a structure file** (see `scripts/example_structure.md` for syntax):

```markdown
Static

##12g Silica Column Bracket
####25001A.U06.P18 12g Silica Column Bracket

##Bin Slot
####25001A.U06.P12 Bin Slot

Dynamic

##12g Silica Column
####12g Silica Column
```

- Plain text lines → top-level group names
- `##` lines → sub-group names
- `####` lines → components to move into the sub-group above

**Run the restructure:**

```bash
python scripts/1_restructure_step_hierarchy.py \
  my_models/front_shelf_en.step \
  --config scripts/example_structure.md \
  --output output/front_shelf_restructured.step
```

---

## 5. Extract component positions

After restructuring, extract world-space positions and orientations for each component:

```bash
python scripts/get_step_component_positions.py output/front_shelf_restructured.step
```

Writes `positions_output.json` and `position.csv`.

---

## Project layout

```
webapp/                 FastAPI app — STEP repair UI and API
  main.py               Routes: GET / (UI), POST /repair
  step_repair.py        Repair logic (syntax fix + Z-axis flip)
  step_parser.py        STEP entity parser, matrix math
  static/index.html     Browser UI
  tests/                pytest suite

scripts/
  0_get_step_component_names.py
  1_restructure_step_hierarchy.py
  translate_step_cn_to_en.py
  get_step_component_positions.py
  extract_positions.py
  read_step_tree.py

config/
  cn_en_translations.json   persistent Chinese→English lookup table

my_models/              input STEP files
output/                 processed STEP files
```
