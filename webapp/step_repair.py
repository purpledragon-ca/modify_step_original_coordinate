"""
step_repair.py — Automatic repair of common Shapr3D STEP export defects.

Two defects are corrected:

1. Bare-integer REAL literals in DIRECTION vectors.
   STEP-21 requires decimal notation for REAL values (e.g. `0.` not `-0` or `1`).
   Shapr3D emits tokens like `-0`, `0`, `1`, `-1` without decimal points.
   Strict parsers (OpenCASCADE XDE reader) reject them, reporting "Incorrect Syntax".

2. Z-down global coordinate frame.
   Shapr3D sets TS3D_PRODUCT_CSYS Z = (0,0,-1).  Every assembly-level
   ITEM_DEFINED_TRANSFORMATION references this frame, so the whole assembly
   arrives with Z pointing downward in Isaac Sim / Shapr3D re-import (both
   Z-up).  This causes components to appear flipped or in wrong orientations.

   Fix: flip the Z direction of TS3D_PRODUCT_CSYS and every matching assembly
   "from"-placement from (0,0,-1) to (0,0,1).  Because both sides of every
   ITEM_DEFINED_TRANSFORMATION are updated together the relative positions of
   parts do not change; only the global frame is declared as Z-up.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Fix 1 — REAL literal normalisation
# ---------------------------------------------------------------------------

_DIRECTION_TUPLE = re.compile(r"DIRECTION\s*\(''\s*,\s*\(([^)]+)\)\s*\)")


def _to_real(tok: str) -> str:
    """Convert a bare integer STEP token to a valid REAL literal."""
    tok = tok.strip()
    if "." in tok or "e" in tok.lower():
        return tok                    # already valid
    if re.fullmatch(r"-0+", tok):
        return "0."                   # -0, -00 … → 0.
    return tok + "."                  # 1 → 1.,  -1 → -1.,  0 → 0.


def _fix_direction_tuple(m: re.Match) -> str:
    fixed = ",".join(_to_real(t) for t in m.group(1).split(","))
    return f"DIRECTION('',({fixed}))"


def fix_direction_literals(line: str) -> str:
    """Fix every DIRECTION('',(...)) token on *line*, return corrected line."""
    return _DIRECTION_TUPLE.sub(_fix_direction_tuple, line)


# ---------------------------------------------------------------------------
# Fix 2 — Z-axis orientation
# ---------------------------------------------------------------------------

# Matches a DIRECTION entity whose Z component is negative-one (any REAL form).
_Z_DOWN_LINE = re.compile(
    r"^(#(\d+)=DIRECTION\('',"
    r"\(\s*-?0\.?\s*,\s*0\.?\s*,\s*-1\.?\s*\)\s*\);)\s*$"
)

# Matches ITEM_DEFINED_TRANSFORMATION('','',#FROM,#TO)
_IDT = re.compile(
    r"ITEM_DEFINED_TRANSFORMATION\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,"
    r"\s*#(\d+)\s*,\s*#(\d+)\s*\)"
)

# Matches AXIS2_PLACEMENT_3D to extract (origin_id, z_dir_id, x_dir_id)
_AX2 = re.compile(
    r"AXIS2_PLACEMENT_3D\s*\('[^']*',\s*#(\d+)\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)"
)


def _collect_entities(lines: list[str]) -> dict[str, str]:
    """
    Single-pass entity map: {entity_id_str: normalised_record_text}.
    Handles both single-line (#N=...;) and multi-line compound entities.
    """
    entities: dict[str, str] = {}
    buf: list[str] = []
    current: str | None = None

    for raw in lines:
        s = raw.strip()
        # New top-level entity starts with #N=
        m = re.match(r"#(\d+)\s*=\s*(.*)", s, re.DOTALL)
        if m:
            if current and buf:
                entities[current] = " ".join(buf)
            current = m.group(1)
            buf = [m.group(2).rstrip(";")]
            if s.endswith(";"):
                entities[current] = " ".join(buf)
                current = None
                buf = []
        elif current:
            buf.append(s.rstrip(";"))
            if s.endswith(";"):
                entities[current] = " ".join(buf)
                current = None
                buf = []

    if current and buf:
        entities[current] = " ".join(buf)

    return entities


def _ids_to_flip(entities: dict[str, str]) -> set[str]:
    """
    Return the set of DIRECTION entity IDs whose Z should be flipped.

    Targets:
    - The Z-direction of TS3D_PRODUCT_CSYS (referenced as 'to' in all IDTs).
    - The Z-direction of every 'from' AXIS2_PLACEMENT_3D in every IDT, when
      that Z-direction is currently pointing down.
    """
    # Build ax2_id → (origin_id, z_id, x_id)
    ax2_map: dict[str, tuple[str, str, str]] = {}
    for eid, rec in entities.items():
        m = _AX2.search(rec)
        if m:
            ax2_map[eid] = (m.group(1), m.group(2), m.group(3))

    # Collect 'from' ax2 IDs from every ITEM_DEFINED_TRANSFORMATION
    from_ids: set[str] = set()
    to_ids: set[str] = set()
    for rec in entities.values():
        m = _IDT.search(rec)
        if m:
            from_ids.add(m.group(1))
            to_ids.add(m.group(2))

    # Also include any ax2 that is the 'to' (captures TS3D_PRODUCT_CSYS)
    candidate_ax2 = from_ids | to_ids

    flip: set[str] = set()
    for ax2_id in candidate_ax2:
        if ax2_id not in ax2_map:
            continue
        _, z_id, _ = ax2_map[ax2_id]
        z_rec = entities.get(z_id, "")
        # Accept any form: -0,0,-1  or  0.,0.,-1.  etc.
        if re.search(r"DIRECTION\s*\(''\s*,\s*\(\s*-?0\.?\s*,\s*0\.?\s*,\s*-1\.?\s*\)\s*\)", z_rec):
            flip.add(z_id)

    return flip


def flip_assembly_z(lines: list[str]) -> tuple[list[str], int]:
    """
    Flip assembly-level Z-down DIRECTION lines to Z-up in-place.
    Returns (new_lines, count_flipped).
    """
    entities = _collect_entities(lines)
    flip_ids = _ids_to_flip(entities)
    if not flip_ids:
        return lines, 0

    result: list[str] = []
    count = 0
    for line in lines:
        m = _Z_DOWN_LINE.match(line.rstrip("\n").rstrip("\r"))
        if m and m.group(2) in flip_ids:
            result.append(f"#{m.group(2)}=DIRECTION('',(0.,0.,1.));\n")
            count += 1
        else:
            result.append(line)
    return result, count


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

@dataclass
class RepairReport:
    syntax_lines_fixed: int = 0
    z_directions_flipped: int = 0

    @property
    def any_changes(self) -> bool:
        return self.syntax_lines_fixed > 0 or self.z_directions_flipped > 0

    def summary(self) -> str:
        parts = []
        if self.syntax_lines_fixed:
            parts.append(
                f"{self.syntax_lines_fixed} line(s) had bare-integer REAL literals fixed"
            )
        if self.z_directions_flipped:
            parts.append(
                f"{self.z_directions_flipped} assembly Z-direction(s) flipped to Z-up"
            )
        return "; ".join(parts) if parts else "No changes needed"


def repair(text: str, *, fix_syntax: bool = True, fix_z: bool = True) -> tuple[str, RepairReport]:
    """
    Repair a STEP file text string.

    Returns:
        (repaired_text, report)
    """
    report = RepairReport()
    lines = text.splitlines(keepends=True)

    if fix_syntax:
        fixed = [fix_direction_literals(l) for l in lines]
        report.syntax_lines_fixed = sum(1 for a, b in zip(lines, fixed) if a != b)
        lines = fixed

    if fix_z:
        lines, n = flip_assembly_z(lines)
        report.z_directions_flipped = n

    return "".join(lines), report
