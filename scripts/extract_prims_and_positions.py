"""
Extract prim paths and world positions from a STEP assembly file.

Walks the full assembly hierarchy recursively, building USD-style prim paths
(/Root/Part/SubPart/...) and computing world-space positions from the BREP
geometry bounding box (the actual position of each component in world space).

For assembly nodes with no direct geometry the bbox is inherited from their
children, so every node in the tree gets a meaningful world position.

Usage:
    python extract_prims_and_positions.py <path_to.step> [options]

Options:
    --csv [FILE]    Save results to CSV (default name: <step>_prims.csv)
    --depth N       Limit recursion depth (default: unlimited)

Example:
    python extract_prims_and_positions.py ../my_models/fraction_waste_bin.step
    python extract_prims_and_positions.py ../my_models/fraction_waste_bin.step --csv
    python extract_prims_and_positions.py ../my_models/fraction_waste_bin.step --csv out.csv
"""

import re
import sys
import csv
import argparse
from pathlib import Path
from collections import defaultdict


# ---------------------------------------------------------------------------
# STEP parsing
# ---------------------------------------------------------------------------

def decode_step_string(s: str) -> str:
    def replace(m):
        hex_chars = m.group(1)
        try:
            return ''.join(chr(int(hex_chars[i:i+4], 16)) for i in range(0, len(hex_chars), 4))
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
# Geometry: cartesian point extraction + bounding box
# ---------------------------------------------------------------------------

def collect_cart_points(entities: dict) -> dict:
    """Return {eid: [x, y, z]} for every CARTESIAN_POINT with 3 coords."""
    cart_points = {}
    for eid, body in entities.items():
        if body.startswith('CARTESIAN_POINT'):
            m = re.match(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]+)\)\s*\)", body)
            if m:
                try:
                    coords = [float(x.strip()) for x in m.group(1).split(',')]
                    if len(coords) == 3:
                        cart_points[eid] = coords
                except ValueError:
                    pass
    return cart_points


def build_ref_graph(entities: dict, cart_points: dict) -> dict:
    """Forward-reference graph used for BFS geometry traversal."""
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
        if any(body.startswith(s) for s in skip):
            continue
        refs = ['#' + r for r in ref_pat.findall(body)]
        if refs:
            graph[eid] = refs
    return graph


def bbox_from_brep(start_id: str, ref_graph: dict, cart_points: dict):
    """BFS from start_id; return (cx,cy,cz) or None if no points found."""
    visited, queue, pts = set(), [start_id], []
    while queue:
        curr = queue.pop()
        if curr in visited:
            continue
        visited.add(curr)
        if curr in cart_points:
            pts.append(cart_points[curr])
            continue
        if curr in ref_graph:
            queue.extend(r for r in ref_graph[curr] if r not in visited)
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return (
        (min(xs) + max(xs)) / 2,
        (min(ys) + max(ys)) / 2,
        (min(zs) + max(zs)) / 2,
    )


def build_pd_to_world_pos(entities: dict) -> dict:
    """
    Map each product_definition id to its world-space bounding-box center.

    Chain walked:
      PRODUCT_DEFINITION
        → PRODUCT_DEFINITION_SHAPE (the one pointing to PD directly)
        → SHAPE_DEFINITION_REPRESENTATION
        → SHAPE_REPRESENTATION
        → SHAPE_REPRESENTATION_RELATIONSHIP → ADVANCED_BREP_SHAPE_REPRESENTATION
        → BFS for CARTESIAN_POINTs → bbox center
    """
    cart_points = collect_cart_points(entities)
    ref_graph   = build_ref_graph(entities, cart_points)

    prod_defs = {eid for eid, v in entities.items() if v.startswith('PRODUCT_DEFINITION(')}

    # PDS that point directly to a PRODUCT_DEFINITION (not via NAUO)
    pds_to_pd = {}
    nauo_ids  = {eid for eid, v in entities.items()
                 if v.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE(')}
    for eid, body in entities.items():
        if body.startswith('PRODUCT_DEFINITION_SHAPE('):
            args = parse_args_top(body)
            if len(args) >= 3:
                ref = args[2]
                if ref in prod_defs:   # points directly to PD, not NAUO
                    pds_to_pd[eid] = ref

    # SDR: pds → shape_rep
    sdr = {}
    for eid, body in entities.items():
        if body.startswith('SHAPE_DEFINITION_REPRESENTATION('):
            m = re.match(r"SHAPE_DEFINITION_REPRESENTATION\s*\(\s*(#\d+)\s*,\s*(#\d+)\s*\)", body)
            if m and m.group(1) in pds_to_pd:
                sdr[m.group(1)] = m.group(2)

    # SHAPE_REPRESENTATION_RELATIONSHIP: shape_rep → advanced_brep
    adv_breps = {eid for eid, v in entities.items()
                 if v.startswith('ADVANCED_BREP_SHAPE_REPRESENTATION')}
    sr_to_adv = {}
    for eid, body in entities.items():
        if body.startswith('SHAPE_REPRESENTATION_RELATIONSHIP('):
            refs = re.findall(r'#\d+', body)
            if len(refs) >= 2 and refs[1] in adv_breps:
                sr_to_adv[refs[0]] = refs[1]
            elif len(refs) >= 2 and refs[0] in adv_breps:
                sr_to_adv[refs[1]] = refs[0]

    pd_to_pos = {}
    for pds_id, pd_id in pds_to_pd.items():
        sr_id  = sdr.get(pds_id)
        if sr_id is None:
            continue
        adv_id = sr_to_adv.get(sr_id)
        if adv_id is None:
            continue
        pos = bbox_from_brep(adv_id, ref_graph, cart_points)
        if pos is not None:
            pd_to_pos[pd_id] = pos

    return pd_to_pos


# ---------------------------------------------------------------------------
# Product name resolution
# ---------------------------------------------------------------------------

def build_pd_name_map(entities: dict) -> dict:
    products = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT('):
            args = parse_args_top(val)
            products[eid] = decode_step_string(args[0].strip("'") or args[1].strip("'"))

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


# ---------------------------------------------------------------------------
# Assembly hierarchy traversal
# ---------------------------------------------------------------------------

def get_nauo_children(parent_pd: str, entities: dict) -> list:
    children = []
    for eid, val in entities.items():
        if val.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE('):
            args = parse_args_top(val)
            if len(args) >= 5 and args[3] == parent_pd:
                children.append({
                    'nauo_id':  eid,
                    'name':     decode_step_string(args[1].strip("'")),
                    'child_pd': args[4],
                })
    return children


def find_root_pd(entities: dict) -> str:
    products, form_to_prod = {}, {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT('):
            args = parse_args_top(val)
            products[eid] = decode_step_string(args[0].strip("'"))
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION_FORMATION'):
            refs = re.findall(r'#\d+', val)
            if refs:
                form_to_prod[eid] = refs[-1]
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION('):
            for ref in re.findall(r'#\d+', val):
                if ref in form_to_prod:
                    if products.get(form_to_prod[ref], '').lower() == 'root':
                        return eid
                    break
    # fallback: PD that is never a child
    all_pds  = {eid for eid, v in entities.items() if v.startswith('PRODUCT_DEFINITION(')}
    child_pds = set()
    for eid, val in entities.items():
        if val.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE('):
            args = parse_args_top(val)
            if len(args) >= 5:
                child_pds.add(args[4])
    roots = all_pds - child_pds
    return next(iter(roots)) if roots else None


def walk(parent_pd: str, entities: dict, pd_to_name: dict,
         pd_to_pos: dict, parent_path: str,
         depth: int, max_depth: int, visited: set) -> list:
    """
    Recursively walk the NAUO tree.
    Returns list of dicts with prim_path, name, tx, ty, tz.
    """
    if max_depth is not None and depth > max_depth:
        return []
    results = []
    for child in get_nauo_children(parent_pd, entities):
        child_pd = child['child_pd']
        if child_pd in visited:
            continue

        raw_name  = pd_to_name.get(child_pd, child['name'] or f'prim_{child_pd[1:]}')
        safe_name = re.sub(r'[/ ]+', '_', raw_name).strip('_') or f'prim_{child_pd[1:]}'
        prim_path = f"{parent_path}/{safe_name}"

        # World position from geometry bbox; None for pure assembly nodes
        pos = pd_to_pos.get(child_pd)

        results.append({
            'prim_path': prim_path,
            'name':      raw_name,
            'tx':        pos[0] if pos else None,
            'ty':        pos[1] if pos else None,
            'tz':        pos[2] if pos else None,
        })

        results.extend(walk(
            child_pd, entities, pd_to_name, pd_to_pos,
            prim_path, depth + 1, max_depth, visited | {child_pd},
        ))
    return results


# ---------------------------------------------------------------------------
# Aggregate positions for assembly nodes
# ---------------------------------------------------------------------------

def fill_assembly_positions(results: list) -> list:
    """
    For nodes without direct geometry (assembly containers), compute their
    world position as the average of all descendent leaf positions.
    """
    # Build child->parent index from prim paths
    pos_by_path = {r['prim_path']: (r['tx'], r['ty'], r['tz'])
                   for r in results if r['tx'] is not None}

    def ancestor_avg(path):
        # collect all descendants that have a position
        prefix = path + '/'
        pts = [v for k, v in pos_by_path.items() if k.startswith(prefix)]
        if not pts:
            return None
        return (
            sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts),
            sum(p[2] for p in pts) / len(pts),
        )

    filled = []
    for r in results:
        if r['tx'] is None:
            avg = ancestor_avg(r['prim_path'])
            if avg:
                r = dict(r, tx=avg[0], ty=avg[1], tz=avg[2])
        filled.append(r)
    return filled


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def print_table(results: list):
    header = f"{'Prim Path':<72} {'tx (mm)':>12} {'ty (mm)':>12} {'tz (mm)':>12}"
    print(header)
    print('-' * len(header))
    for r in results:
        tx = f"{r['tx']:>12.3f}" if r['tx'] is not None else f"{'—':>12}"
        ty = f"{r['ty']:>12.3f}" if r['ty'] is not None else f"{'—':>12}"
        tz = f"{r['tz']:>12.3f}" if r['tz'] is not None else f"{'—':>12}"
        print(f"{r['prim_path']:<72} {tx} {ty} {tz}")
    print(f"\nTotal prims: {len(results)}")


def save_csv(results: list, path: Path):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['prim_path', 'name', 'tx_mm', 'ty_mm', 'tz_mm'])
        for r in results:
            writer.writerow([
                r['prim_path'], r['name'],
                f"{r['tx']:.4f}" if r['tx'] is not None else '',
                f"{r['ty']:.4f}" if r['ty'] is not None else '',
                f"{r['tz']:.4f}" if r['tz'] is not None else '',
            ])
    print(f"Saved {len(results)} prims to {path}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Extract prim paths and world positions from a STEP assembly file.')
    parser.add_argument('step_file', help='Path to the .step file')
    parser.add_argument('--csv', metavar='FILE', nargs='?', const='',
                        help='Save to CSV (default name: <step>_prims.csv)')
    parser.add_argument('--depth', type=int, default=None,
                        help='Max recursion depth (default: unlimited)')
    args = parser.parse_args()

    path = Path(args.step_file)
    if not path.exists():
        print(f"Error: file not found: {path}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading: {path.name}", file=sys.stderr)
    text = path.read_text(encoding='utf-8', errors='replace')

    print("Parsing entities...", file=sys.stderr)
    entities = parse_step_entities(text)
    print(f"  {len(entities)} entities loaded", file=sys.stderr)

    print("Building geometry positions...", file=sys.stderr)
    pd_to_pos  = build_pd_to_world_pos(entities)
    pd_to_name = build_pd_name_map(entities)

    root_pd = find_root_pd(entities)
    if root_pd is None:
        print("Error: could not determine root assembly.", file=sys.stderr)
        sys.exit(1)

    root_name = pd_to_name.get(root_pd, 'Root')
    safe_root = re.sub(r'[/ ]+', '_', root_name).strip('_') or 'Root'
    print(f"  Root: {root_pd}  ({root_name})", file=sys.stderr)

    results = walk(
        parent_pd=root_pd,
        entities=entities,
        pd_to_name=pd_to_name,
        pd_to_pos=pd_to_pos,
        parent_path=f"/{safe_root}",
        depth=1,
        max_depth=args.depth,
        visited={root_pd},
    )

    results = fill_assembly_positions(results)
    print(f"  Found {len(results)} prims\n", file=sys.stderr)

    print_table(results)

    if args.csv is not None:
        csv_path = Path(args.csv) if args.csv else path.with_name(path.stem + '_prims.csv')
        save_csv(results, csv_path)


if __name__ == '__main__':
    main()
