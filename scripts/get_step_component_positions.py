"""
Extract the position (bounding box center) of each component from a STEP file.

Usage:
    python get_step_component_positions.py <path_to_step_file> [options]

Options:
    --csv         Output results as CSV
    --sort-by     Sort by: name, x, y, z (default: name)
    --filter      Filter component names by substring

Example:
    python get_step_component_positions.py ../isaacsim_models/left_shelf.step
    python get_step_component_positions.py ../isaacsim_models/left_shelf.step --csv
    python get_step_component_positions.py ../isaacsim_models/left_shelf.step --filter 过滤柱
"""

import re
import sys
import argparse
import csv
import io
from pathlib import Path


def decode_step_string(s):
    """Decode STEP \\X2\\...\\X0\\ Unicode escape sequences."""
    def replace_unicode(m):
        hex_str = m.group(1)
        try:
            chars = [chr(int(hex_str[i:i+4], 16)) for i in range(0, len(hex_str), 4)]
            return ''.join(chars)
        except Exception:
            return m.group(0)
    return re.sub(r'\\X2\\([0-9A-Fa-f]+)\\X0\\', replace_unicode, s)


def parse_records(content):
    """Parse all STEP entity records into a dict: entity_id -> body string."""
    records = {}
    for match in re.finditer(r'#(\d+)\s*=\s*(.+?);', content, re.DOTALL):
        num = int(match.group(1))
        body = match.group(2).replace('\n', ' ').strip()
        records[num] = body
    return records


def build_forward_refs(records, cart_points):
    """Build a forward reference graph for BFS traversal (skip geometric leaves)."""
    ref_pattern = re.compile(r'#(\d+)')
    skip_starts = (
        'CARTESIAN_POINT', 'DIRECTION', 'VECTOR', 'UNCERTAINTY_MEASURE',
        'GEOMETRIC_REPRESENTATION_CONTEXT', 'PLANE_ANGLE_MEASURE',
        'DIMENSIONAL_EXPONENTS', 'CONVERSION_BASED_UNIT', 'NAMED_UNIT',
        'LENGTH_UNIT', 'SI_UNIT', 'SOLID_ANGLE_UNIT',
    )
    forward_refs = {}
    for num, body in records.items():
        if num in cart_points:
            continue
        if any(body.startswith(s) for s in skip_starts):
            continue
        refs = [int(m) for m in ref_pattern.findall(body)]
        if refs:
            forward_refs[num] = refs
    return forward_refs


def get_points_bfs(start_id, forward_refs, cart_points):
    """BFS from a start entity to collect all reachable CARTESIAN_POINT coordinates."""
    visited = set()
    queue = [start_id]
    pts = []
    while queue:
        curr = queue.pop()
        if curr in visited:
            continue
        visited.add(curr)
        if curr in cart_points:
            pts.append(cart_points[curr])
            continue
        if curr in forward_refs:
            queue.extend(r for r in forward_refs[curr] if r not in visited)
    return pts


def extract_component_positions(step_file_path):
    """
    Parse a STEP file and return a list of dicts with component names and positions.

    Each dict contains:
        name    : component name (decoded from STEP)
        center  : (cx, cy, cz) bounding box center in mm
        bbox    : (xmin, xmax, ymin, ymax, zmin, zmax) in mm
    """
    path = Path(step_file_path)
    if not path.exists():
        raise FileNotFoundError(f"STEP file not found: {step_file_path}")

    print(f"Reading: {path.name}", file=sys.stderr)
    content = path.read_text(encoding='utf-8', errors='replace')

    print("Parsing records...", file=sys.stderr)
    records = parse_records(content)
    print(f"  Total records: {len(records)}", file=sys.stderr)

    ref_pattern = re.compile(r'#(\d+)')

    # --- Cartesian points ---
    cart_points = {}
    for num, body in records.items():
        if body.startswith('CARTESIAN_POINT'):
            m = re.match(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]+)\)\s*\)", body)
            if m:
                try:
                    cart_points[num] = [float(x.strip()) for x in m.group(1).split(',')]
                except ValueError:
                    pass

    # --- Product name chain ---
    # PRODUCT_DEFINITION -> PRODUCT_DEFINITION_FORMATION -> PRODUCT (name)
    products = {}
    for num, body in records.items():
        if body.startswith('PRODUCT('):
            m = re.match(r"PRODUCT\s*\(\s*'([^']*)'\s*,\s*'([^']*)'", body)
            if m:
                products[num] = decode_step_string(m.group(1) or m.group(2))

    prod_form_to_prod = {}
    for num, body in records.items():
        if body.startswith('PRODUCT_DEFINITION_FORMATION'):
            m = re.match(
                r"PRODUCT_DEFINITION_FORMATION[A-Z_]*\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,\s*(#\d+)",
                body
            )
            if m:
                prod_form_to_prod[num] = int(m.group(1)[1:])

    prod_def_to_name = {}
    for num, body in records.items():
        if body.startswith('PRODUCT_DEFINITION('):
            for ref in [int(r) for r in ref_pattern.findall(body)]:
                if ref in prod_form_to_prod:
                    prod_id = prod_form_to_prod[ref]
                    prod_def_to_name[num] = products.get(prod_id, f'unknown_{prod_id}')
                    break

    # --- Shape representation chain ---
    # PDS -> PRODUCT_DEFINITION
    prod_defs = {n for n, b in records.items() if b.startswith('PRODUCT_DEFINITION(')}

    pds_to_prod_def = {}
    for num, body in records.items():
        if body.startswith('PRODUCT_DEFINITION_SHAPE'):
            m = re.match(
                r"PRODUCT_DEFINITION_SHAPE\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,\s*(#\d+)\s*\)",
                body
            )
            if m:
                ref = int(m.group(1)[1:])
                if ref in prod_defs:
                    pds_to_prod_def[num] = ref

    # SHAPE_DEFINITION_REPRESENTATION: pds -> shape_rep
    sdr_map = {}
    for num, body in records.items():
        if body.startswith('SHAPE_DEFINITION_REPRESENTATION'):
            m = re.match(
                r"SHAPE_DEFINITION_REPRESENTATION\s*\(\s*(#\d+)\s*,\s*(#\d+)\s*\)",
                body
            )
            if m:
                sdr_map[int(m.group(1)[1:])] = int(m.group(2)[1:])

    # SHAPE_REPRESENTATION_RELATIONSHIP: shape_rep -> advanced_brep
    adv_breps = {n for n, b in records.items() if b.startswith('ADVANCED_BREP_SHAPE_REPRESENTATION')}
    shape_rep_to_adv = {}
    for num, body in records.items():
        if body.startswith('SHAPE_REPRESENTATION_RELATIONSHIP'):
            refs = [int(r) for r in ref_pattern.findall(body)]
            if len(refs) >= 2 and refs[1] in adv_breps:
                shape_rep_to_adv[refs[0]] = refs[1]

    # --- Build BFS reference graph ---
    print("Building reference graph...", file=sys.stderr)
    forward_refs = build_forward_refs(records, cart_points)

    # --- Compute bounding boxes ---
    print("Computing component positions...", file=sys.stderr)
    results = []
    for pds_num, prod_def in pds_to_prod_def.items():
        shape_rep = sdr_map.get(pds_num)
        if shape_rep is None:
            continue
        adv_brep = shape_rep_to_adv.get(shape_rep)
        if adv_brep is None:
            continue  # assembly node with no direct geometry

        pts = get_points_bfs(adv_brep, forward_refs, cart_points)
        if not pts:
            continue

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        zmin, zmax = min(zs), max(zs)
        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2
        cz = (zmin + zmax) / 2

        results.append({
            'name': prod_def_to_name.get(prod_def, f'#pd{prod_def}'),
            'center': (cx, cy, cz),
            'bbox': (xmin, xmax, ymin, ymax, zmin, zmax),
        })

    return results


def print_table(results):
    header = f"{'#':<4} {'Component Name':<55} {'Center X (mm)':>14} {'Center Y (mm)':>14} {'Center Z (mm)':>14}"
    print(header)
    print('-' * len(header))
    for i, r in enumerate(results, 1):
        cx, cy, cz = r['center']
        print(f"{i:<4} {r['name']:<55} {cx:>14.1f} {cy:>14.1f} {cz:>14.1f}")


def print_csv(results):
    writer = csv.writer(sys.stdout)
    writer.writerow(['#', 'name', 'center_x_mm', 'center_y_mm', 'center_z_mm',
                     'xmin_mm', 'xmax_mm', 'ymin_mm', 'ymax_mm', 'zmin_mm', 'zmax_mm'])
    for i, r in enumerate(results, 1):
        cx, cy, cz = r['center']
        xmin, xmax, ymin, ymax, zmin, zmax = r['bbox']
        writer.writerow([i, r['name'], f'{cx:.3f}', f'{cy:.3f}', f'{cz:.3f}',
                         f'{xmin:.3f}', f'{xmax:.3f}', f'{ymin:.3f}', f'{ymax:.3f}',
                         f'{zmin:.3f}', f'{zmax:.3f}'])


def main():
    parser = argparse.ArgumentParser(
        description='Extract component positions from a STEP file.'
    )
    parser.add_argument('step_file', help='Path to the .step / .STEP file')
    parser.add_argument('--csv', action='store_true', help='Output as CSV')
    parser.add_argument('--output', metavar='FILE', default='position.csv',
                        help='Save CSV output to FILE (default: position.csv); implies --csv')
    parser.add_argument('--sort-by', choices=['name', 'x', 'y', 'z'], default='name',
                        help='Sort results by field (default: name)')
    parser.add_argument('--filter', metavar='TEXT',
                        help='Only show components whose name contains TEXT')
    args = parser.parse_args()

    results = extract_component_positions(args.step_file)

    # Filter
    if args.filter:
        results = [r for r in results if args.filter in r['name']]

    # Sort
    sort_key = {
        'name': lambda r: r['name'],
        'x': lambda r: r['center'][0],
        'y': lambda r: r['center'][1],
        'z': lambda r: r['center'][2],
    }[args.sort_by]
    results.sort(key=sort_key)

    print(f"\nFound {len(results)} components\n", file=sys.stderr)

    if args.output or args.csv:
        out_path = Path(args.output)
        with out_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['#', 'name', 'center_x_mm', 'center_y_mm', 'center_z_mm',
                             'xmin_mm', 'xmax_mm', 'ymin_mm', 'ymax_mm', 'zmin_mm', 'zmax_mm'])
            for i, r in enumerate(results, 1):
                cx, cy, cz = r['center']
                xmin, xmax, ymin, ymax, zmin, zmax = r['bbox']
                writer.writerow([i, r['name'], f'{cx:.3f}', f'{cy:.3f}', f'{cz:.3f}',
                                 f'{xmin:.3f}', f'{xmax:.3f}', f'{ymin:.3f}', f'{ymax:.3f}',
                                 f'{zmin:.3f}', f'{zmax:.3f}'])
        print(f"Saved {len(results)} components to {out_path}", file=sys.stderr)
    else:
        print_table(results)
        print(f"\nTotal: {len(results)} components")
        print("Coordinates are bounding box centers in mm (world/absolute coordinates).")


if __name__ == '__main__':
    main()
