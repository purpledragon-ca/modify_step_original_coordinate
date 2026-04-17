"""
Translate Chinese part names in a STEP file to English.

All Chinese text in STEP files is encoded as \\X2\\<hex>\\X0\\.

Workflow:
  1. Load master table from config/cn_en_translations.json.
  2. If a <stem>_cn_en_comparison.csv exists next to the STEP file, read it
     and compare with the master table — print any mismatches.
  3. Merge new entries from the CSV into the master table and save.
  4. Replace all encoded Chinese strings in the STEP file and write <stem>_en.step.
  5. Any encoded strings still not in the table are appended as placeholders.

Usage:
    python scripts/translate_step_cn_to_en.py my_models/front_shelf.step
"""

import csv
import json
import re
import sys
from pathlib import Path

CONFIG_PATH = Path(__file__).parent.parent / "config" / "cn_en_translations.json"


def load_config() -> dict:
    if CONFIG_PATH.exists():
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return {}


def save_config(cn_to_en: dict) -> None:
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(
        json.dumps(cn_to_en, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def load_comparison_csv(csv_path: Path) -> dict:
    """Read a _cn_en_comparison.csv and return {chinese: english}."""
    result = {}
    with csv_path.open(encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            cn = row.get("chinese", "").strip()
            en = row.get("english", "").strip()
            if cn:
                result[cn] = en
    return result


def decode_x2(hex_str: str) -> str:
    return "".join(chr(int(hex_str[i: i + 4], 16)) for i in range(0, len(hex_str), 4))


def build_replacement_map(cn_to_en: dict) -> dict:
    token_map = {}
    for cn, en in cn_to_en.items():
        if not en:
            continue
        hex_code = "".join(f"{ord(c):04X}" for c in cn)
        token_map[f"\\X2\\{hex_code}\\X0\\"] = en
    return token_map


def check_mismatches(config: dict, csv_data: dict) -> None:
    mismatches = []
    for cn, csv_en in csv_data.items():
        cfg_en = config.get(cn)
        if cfg_en is not None and cfg_en != csv_en:
            mismatches.append((cn, cfg_en, csv_en))

    if mismatches:
        print(f"\n[MISMATCH] {len(mismatches)} entry(ies) differ between CSV and config:")
        print(f"  {'Chinese':<30}  {'Config English':<40}  CSV English")
        print(f"  {'-'*30}  {'-'*40}  {'-'*40}")
        for cn, cfg_en, csv_en in mismatches:
            print(f"  {cn:<30}  {cfg_en:<40}  {csv_en}")
    else:
        print("No mismatches between comparison CSV and config.")


def translate_step(src_path: Path) -> None:
    config = load_config()

    # --- Step 1: load and compare existing comparison CSV ---
    csv_path = src_path.with_name(src_path.stem + "_cn_en_comparison.csv")
    if csv_path.exists():
        print(f"Found comparison CSV: {csv_path}")
        csv_data = load_comparison_csv(csv_path)
        check_mismatches(config, csv_data)
        # Merge CSV entries into config (CSV values win for new keys only)
        added = {cn: en for cn, en in csv_data.items() if cn not in config}
        if added:
            print(f"\nMerging {len(added)} new entry(ies) from CSV into config:")
            for cn, en in added.items():
                print(f"  {cn}  →  {en}")
            config.update(added)
            save_config(config)
            print(f"Saved → {CONFIG_PATH}")
    else:
        print(f"No comparison CSV found at {csv_path}, skipping CSV check.")

    # --- Step 2: translate the STEP file ---
    text = src_path.read_text(encoding="utf-8", errors="replace")
    token_map = build_replacement_map(config)

    for token in sorted(token_map, key=len, reverse=True):
        text = text.replace(token, token_map[token])

    # --- Step 3: report / save any untranslated strings ---
    remaining_hex = re.findall(r"\\X2\\([0-9A-Fa-f]+)\\X0\\", text)
    if remaining_hex:
        new_entries = {}
        for hex_str in sorted(set(remaining_hex)):
            decoded = decode_x2(hex_str)
            if decoded not in config:
                new_entries[decoded] = ""
        if new_entries:
            print(f"\n[TODO] {len(new_entries)} untranslated string(s) added to config (fill in English):")
            for cn in new_entries:
                print(f"  {cn!r}")
            config.update(new_entries)
            save_config(config)
            print(f"Saved → {CONFIG_PATH}")
    else:
        print("\nAll X2-encoded strings replaced successfully.")

    en_path = src_path.with_name(src_path.stem + "_en.step")
    en_path.write_text(text, encoding="utf-8")
    print(f"English STEP written: {en_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python translate_step_cn_to_en.py <file.step>")
        sys.exit(1)
    src = Path(sys.argv[1])
    if not src.exists():
        print(f"Error: file not found: {src}")
        sys.exit(1)
    translate_step(src)


if __name__ == "__main__":
    main()
