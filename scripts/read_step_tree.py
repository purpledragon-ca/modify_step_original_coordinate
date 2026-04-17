"""
STEP assembly tree reader.

Parses a STEP file and prints its full product assembly hierarchy as a tree.

Usage:
    python scripts/read_step_tree.py path/to/file.step
"""

import re
import sys
import argparse
from pathlib import Path
from collections import defaultdict

# ---------------------------------------------------------------------------
# STEP entity parsing helpers (copied from read_step_poses.py)
# ---------------------------------------------------------------------------

def parse_step_entities(text: str) -> dict:
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    m = re.search(r'\bDATA\s*;(.*?)\bENDSEC\b', text, re.DOTALL | re.IGNORECASE)
    if not m:
        raise ValueError("No DATA section found in STEP file")
    data_section = m.group(1)

    entities = {}
    for stmt in data_section.split(';'):
        stmt = stmt.strip()
        if not stmt:
            continue
        m2 = re.match(r'(#\d+)\s*=\s*(.*)', stmt, re.DOTALL)
        if m2:
            eid = m2.group(1)
            val = ' '.join(m2.group(2).split())
            entities[eid] = val
    return entities


def decode_step_string(s: str) -> str:
    def replace(m):
        hex_chars = m.group(1)
        try:
            pairs = [hex_chars[i:i+4] for i in range(0, len(hex_chars), 4)]
            return ''.join(chr(int(p, 16)) for p in pairs)
        except Exception:
            return m.group(0)
    return re.sub(r'\\X2\\([0-9A-Fa-f]+)\\X0\\', replace, s)


def parse_args_top(val: str) -> list:
    m = re.match(r'[A-Z_0-9]*\((.*)\)\s*$', val, re.DOTALL)
    if not m:
        m = re.match(r'\((.*)\)\s*$', val, re.DOTALL)
        if not m:
            return []
    inner = m.group(1)
    args = []
    depth = 0
    current = []
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


def extract_refs(arg: str) -> list:
    return re.findall(r'#\d+', arg)


# ---------------------------------------------------------------------------
# Build product name mapping
# ---------------------------------------------------------------------------

def build_pd_to_product_name(entities: dict) -> dict:
    """
    Returns a dict mapping PRODUCT_DEFINITION id → decoded product name.

    Chain: PRODUCT(name,...) ← PRODUCT_DEFINITION_FORMATION ← PRODUCT_DEFINITION
    """
    # 1. Map PRODUCT id → name
    product_names = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT('):
            args = parse_args_top(val)
            if args:
                product_names[eid] = decode_step_string(args[0].strip("'"))

    # 2. Map PRODUCT_DEFINITION_FORMATION id → PRODUCT id
    pdf_to_product = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION_FORMATION('):
            refs = extract_refs(val)
            if refs:
                pdf_to_product[eid] = refs[-1]

    # 3. Map PRODUCT_DEFINITION id → name via the above chain
    pd_to_name = {}
    for eid, val in entities.items():
        if val.startswith('PRODUCT_DEFINITION('):
            args = parse_args_top(val)
            if len(args) >= 3:
                pdf_ref = args[2]
                prod_ref = pdf_to_product.get(pdf_ref)
                if prod_ref and prod_ref in product_names:
                    pd_to_name[eid] = product_names[prod_ref]
    return pd_to_name


# ---------------------------------------------------------------------------
# Build assembly tree from NAUO relationships
# ---------------------------------------------------------------------------

def build_assembly_tree(entities: dict) -> tuple:
    """
    Returns (children, nauo_names) where:
      children: dict mapping parent PRODUCT_DEFINITION id → list of (child_pd, instance_name)
      nauo_names: dict mapping (parent_pd, child_pd) → instance_name
    """
    children = defaultdict(list)

    for eid, val in entities.items():
        if val.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE('):
            args = parse_args_top(val)
            if len(args) < 5:
                continue
            # NAUO(id, name, description, relating_pd, related_pd, $)
            instance_name = decode_step_string(args[1].strip("'"))
            parent_pd = args[3]
            child_pd = args[4]
            children[parent_pd].append((child_pd, instance_name))

    return children


def find_root_pd(entities: dict, pd_to_name: dict) -> str | None:
    """Find the PRODUCT_DEFINITION whose product name is 'root'."""
    for eid, name in pd_to_name.items():
        if name == 'root':
            return eid
    return None


# ---------------------------------------------------------------------------
# Tree printer
# ---------------------------------------------------------------------------

def print_tree(pd_id: str, pd_to_name: dict, children: dict,
               prefix: str = '', is_last: bool = True, visited: set = None):
    if visited is None:
        visited = set()

    name = pd_to_name.get(pd_id, pd_id)
    connector = '└── ' if is_last else '├── '
    print(prefix + (connector if prefix else '') + name)

    if pd_id in visited:
        # Prevent infinite loops in case of cycles
        return
    visited = visited | {pd_id}

    kids = children.get(pd_id, [])
    child_prefix = prefix + ('    ' if is_last else '│   ')
    for i, (child_pd, _instance_name) in enumerate(kids):
        print_tree(child_pd, pd_to_name, children,
                   prefix=child_prefix,
                   is_last=(i == len(kids) - 1),
                   visited=visited)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Print the assembly tree of a STEP file.')
    parser.add_argument('step_file', help='Path to the STEP file')
    args = parser.parse_args()

    path = Path(args.step_file)
    if not path.exists():
        print(f"Error: file not found: {path}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading: {path}", file=sys.stderr)
    text = path.read_text(encoding='utf-8', errors='replace')

    print("Parsing entities ...", file=sys.stderr)
    entities = parse_step_entities(text)
    print(f"  {len(entities)} entities loaded", file=sys.stderr)

    pd_to_name = build_pd_to_product_name(entities)
    children = build_assembly_tree(entities)
    root_pd = find_root_pd(entities, pd_to_name)

    if root_pd is None:
        print("Error: could not find a PRODUCT named 'root'.", file=sys.stderr)
        sys.exit(1)

    print(f"\nAssembly tree for: {path.name}\n", file=sys.stderr)
    print_tree(root_pd, pd_to_name, children)


if __name__ == '__main__':
    main()
