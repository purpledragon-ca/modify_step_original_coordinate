from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pathlib import Path

from step_repair import repair

app = FastAPI()

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def root():
    return HTMLResponse((STATIC_DIR / "index.html").read_text(encoding="utf-8"))


@app.post("/repair")
async def repair_step(
    file: UploadFile = File(...),
    fix_syntax: bool = Form(True),
    fix_z: bool = Form(True),
):
    if not file.filename.lower().endswith((".step", ".stp")):
        return JSONResponse(status_code=400, content={"error": "File must be a .step or .stp file"})

    raw = await file.read()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return JSONResponse(status_code=400, content={"error": "File is not valid UTF-8 text"})

    repaired, report = repair(text, fix_syntax=fix_syntax, fix_z=fix_z)

    stem = Path(file.filename).stem
    out_name = f"{stem}_repaired.step"

    return Response(
        content=repaired.encode("utf-8"),
        media_type="application/octet-stream",
        headers={
            "Content-Disposition": f'attachment; filename="{out_name}"',
            "X-Repair-Summary": report.summary(),
            "X-Syntax-Lines-Fixed": str(report.syntax_lines_fixed),
            "X-Z-Directions-Flipped": str(report.z_directions_flipped),
        },
    )
