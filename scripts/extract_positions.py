"""
extract_positions.py

Reads out.csv and a components_config.json, finds all prim paths that
match each component's path suffix or name, and writes a positions JSON
suitable for reloading assets in Isaac Sim.

Usage:
    python scripts/extract_positions.py \
        --csv out.csv \
        --config components_config.json \
        --output positions_output.json
"""

import argparse
import csv
import json
from pathlib import Path


def load_csv(csv_path: str) -> list[dict]:
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def find_matches(rows: list[dict], suffix: str) -> list[dict]:
    """Return rows whose prim_path ends with the given suffix."""
    matches = []
    for row in rows:
        if row["prim_path"].endswith(suffix):
            matches.append({
                "prim_path": row["prim_path"],
                "name": row["name"],
                "position_mm": {
                    "x": float(row["tx_mm"]),
                    "y": float(row["ty_mm"]),
                    "z": float(row["tz_mm"]),
                },
                "position_m": {
                    "x": float(row["tx_mm"]) / 1000.0,
                    "y": float(row["ty_mm"]) / 1000.0,
                    "z": float(row["tz_mm"]) / 1000.0,
                },
            })
    return matches


def main():
    parser = argparse.ArgumentParser(description="Extract component positions from out.csv")
    parser.add_argument("--csv", default="out.csv", help="Path to out.csv")
    parser.add_argument("--config", default="components_config.json", help="Path to components config JSON")
    parser.add_argument("--output", default="positions_output.json", help="Path to write output JSON")
    args = parser.parse_args()

    rows = load_csv(args.csv)

    with open(args.config, encoding="utf-8") as f:
        config = json.load(f)

    output = {"components": []}

    for comp in config["components"]:
        suffix = comp["match_path_suffix"]
        matches = find_matches(rows, suffix)
        entry = {
            "label": comp["label"],
            "match_path_suffix": suffix,
            "usd_asset": comp.get("usd_asset", ""),
            "count": len(matches),
            "instances": matches,
        }
        output["components"].append(entry)
        print(f"[{comp['label']}] found {len(matches)} instance(s)")

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2)

    print(f"\nPositions written to: {args.output}")


if __name__ == "__main__":
    main()
