#!/usr/bin/env python3
"""
Find Bottom Center - STEP Model Analysis Tool

Auto-detects bottom center and canonical frame of STEP models.
Supports interactive 3D face selection via web UI.
Exports a re-centred STEP file **and** a transform-record JSON.

Usage:
    python find_bottom_center.py model.step              # Auto-detect, print
    python find_bottom_center.py model.step --ui         # Interactive 3D UI
    python find_bottom_center.py model.step -e out.step  # Auto-export STEP

Requirements:
    pip install cadquery   # provides OCP (OpenCascade Python bindings)
"""

import argparse
import json
import math
import os
import re
import sys
import tempfile
import traceback
import webbrowser
from dataclasses import dataclass, field, asdict
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from typing import List, Optional, Tuple

from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.Interface import Interface_Static
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import (
    GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone,
    GeomAbs_Sphere, GeomAbs_Torus,
)
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.gp import gp_Pnt, gp_Vec, gp_Trsf, gp_Ax1, gp_Dir
from OCP.TopLoc import TopLoc_Location
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopoDS import TopoDS


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class FaceInfo:
    id: int
    area: float
    centroid: Tuple[float, float, float]
    normal: Tuple[float, float, float]
    surface_type: str
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]
    axis_origin: Optional[Tuple[float, float, float]] = None
    axis_direction: Optional[Tuple[float, float, float]] = None
    radius: Optional[float] = None

@dataclass
class Proposal:
    method: str
    label: str
    center: Tuple[float, float, float]
    z_axis: Tuple[float, float, float]
    confidence: float
    face_ids: List[int] = field(default_factory=list)

@dataclass
class JointFeature:
    type: str
    axis_origin: Tuple[float, float, float]
    axis_direction: Tuple[float, float, float]
    face_ids: List[int] = field(default_factory=list)
    radius: Optional[float] = None


# ---------------------------------------------------------------------------
# Vector helpers
# ---------------------------------------------------------------------------

def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def _length(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def _normalize(v):
    n = _length(v)
    return (v[0]/n, v[1]/n, v[2]/n) if n > 1e-12 else v

def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def _sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def _rot_matrix_to_euler_xyz(R):
    """3x3 rotation matrix -> (rx, ry, rz) in degrees (extrinsic XYZ)."""
    ry = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(ry)) > 1e-6:
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    else:
        rx = 0.0
        rz = math.atan2(-R[0][1], R[1][1])
    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


# ---------------------------------------------------------------------------
# Unit detection
# ---------------------------------------------------------------------------

_SI_PREFIX_MAP = {
    ".MILLI.": ("mm", 1.0),
    ".CENTI.": ("cm", 10.0),
    ".MICRO.": ("um", 0.001),
    "$": ("m", 1000.0),      # no prefix = base SI = metre
}

def detect_step_unit(path: str) -> str:
    """Read the STEP header and return the length unit string (mm/m/cm/inch)."""
    with open(path, "r", errors="replace") as f:
        head = f.read(8192)

    # SI_UNIT(.MILLI.,.METRE.)
    m = re.search(r"SI_UNIT\s*\(\s*([^,)]+)\s*,\s*\.METRE\.\s*\)", head)
    if m:
        prefix = m.group(1).strip()
        return _SI_PREFIX_MAP.get(prefix, ("mm", 1.0))[0]

    # CONVERSION_BASED_UNIT('INCH', ...)
    if re.search(r"CONVERSION_BASED_UNIT\s*\(\s*'INCH'", head, re.IGNORECASE):
        return "inch"

    return "mm"   # default


# ---------------------------------------------------------------------------
# STEP loading
# ---------------------------------------------------------------------------

def load_step(path: str):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read STEP file: {path}")
    reader.TransferRoots()
    return reader.OneShape()


# ---------------------------------------------------------------------------
# Face analysis
# ---------------------------------------------------------------------------

def analyze_faces(shape) -> List[FaceInfo]:
    bbox_all = Bnd_Box()
    BRepBndLib.Add_s(shape, bbox_all)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox_all.Get()
    diag = math.sqrt((xmax-xmin)**2 + (ymax-ymin)**2 + (zmax-zmin)**2)
    deflection = max(diag * 0.002, 0.1)
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)

    faces: List[FaceInfo] = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    fid = 0

    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, props)
        area = props.Mass()
        cm = props.CentreOfMass()
        centroid = (cm.X(), cm.Y(), cm.Z())

        fb = Bnd_Box()
        BRepBndLib.Add_s(face, fb)
        x0, y0, z0, x1, y1, z1 = fb.Get()

        adaptor = BRepAdaptor_Surface(face)
        stype_enum = adaptor.GetType()
        normal = (0.0, 0.0, 1.0)
        axis_origin = axis_direction = None
        radius = None

        if stype_enum == GeomAbs_Plane:
            d = adaptor.Plane().Axis().Direction()
            normal = (d.X(), d.Y(), d.Z())
            stype = "plane"
        elif stype_enum == GeomAbs_Cylinder:
            cyl = adaptor.Cylinder()
            d = cyl.Axis().Direction(); loc = cyl.Location()
            normal = (d.X(), d.Y(), d.Z())
            axis_origin = (loc.X(), loc.Y(), loc.Z())
            axis_direction = normal
            radius = cyl.Radius()
            stype = "cylinder"
        elif stype_enum == GeomAbs_Cone:
            cone = adaptor.Cone()
            d = cone.Axis().Direction(); loc = cone.Location()
            normal = (d.X(), d.Y(), d.Z())
            axis_origin = (loc.X(), loc.Y(), loc.Z())
            axis_direction = normal
            stype = "cone"
        elif stype_enum == GeomAbs_Sphere:
            c = adaptor.Sphere().Location()
            dx, dy, dz = centroid[0]-c.X(), centroid[1]-c.Y(), centroid[2]-c.Z()
            n = math.sqrt(dx*dx+dy*dy+dz*dz)
            normal = (dx/n, dy/n, dz/n) if n > 1e-12 else (0, 0, 1)
            stype = "sphere"
        elif stype_enum == GeomAbs_Torus:
            tor = adaptor.Torus()
            d = tor.Axis().Direction(); loc = tor.Location()
            normal = (d.X(), d.Y(), d.Z())
            axis_origin = (loc.X(), loc.Y(), loc.Z())
            axis_direction = normal
            stype = "torus"
        else:
            stype = "other"
            try:
                u = (adaptor.FirstUParameter()+adaptor.LastUParameter())/2
                v = (adaptor.FirstVParameter()+adaptor.LastVParameter())/2
                p = gp_Pnt(); d1u = gp_Vec(); d1v = gp_Vec()
                adaptor.D1(u, v, p, d1u, d1v)
                nv = d1u.Crossed(d1v); mag = nv.Magnitude()
                if mag > 1e-12:
                    normal = (nv.X()/mag, nv.Y()/mag, nv.Z()/mag)
            except Exception:
                pass

        if face.Orientation() == TopAbs_REVERSED:
            normal = (-normal[0], -normal[1], -normal[2])

        faces.append(FaceInfo(
            id=fid, area=area, centroid=centroid, normal=normal,
            surface_type=stype, bbox_min=(x0,y0,z0), bbox_max=(x1,y1,z1),
            axis_origin=axis_origin, axis_direction=axis_direction, radius=radius,
        ))
        fid += 1
        explorer.Next()
    return faces


# ---------------------------------------------------------------------------
# Triangulation (for 3D viewer)
# ---------------------------------------------------------------------------

def triangulate_faces(shape) -> List[dict]:
    meshes = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    fid = 0
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, location)
        if tri is not None:
            tf = location.Transformation()
            rev = (face.Orientation() == TopAbs_REVERSED)
            verts = []
            for i in range(1, tri.NbNodes()+1):
                p = tri.Node(i); p.Transform(tf)
                verts.append((p.X(), p.Y(), p.Z()))
            positions = []; normals = []
            for i in range(1, tri.NbTriangles()+1):
                t = tri.Triangle(i); i1, i2, i3 = t.Get()
                if rev: i1, i3 = i3, i1
                v1, v2, v3 = verts[i1-1], verts[i2-1], verts[i3-1]
                e1 = _sub(v2,v1); e2 = _sub(v3,v1)
                n = _cross(e1,e2); nl = _length(n)
                n = (n[0]/nl,n[1]/nl,n[2]/nl) if nl>1e-15 else (0,0,1)
                for v in (v1,v2,v3):
                    positions.extend(v); normals.extend(n)
            if positions:
                meshes.append({"face_id":fid,"positions":positions,"normals":normals})
        fid += 1; explorer.Next()
    return meshes


# ---------------------------------------------------------------------------
# Method 1: Rule-based bottom center detection
# ---------------------------------------------------------------------------

def find_bottom_center_rules(faces: List[FaceInfo]) -> List[Proposal]:
    proposals: List[Proposal] = []
    if not faces:
        return proposals

    all_z = [f.centroid[2] for f in faces]
    z_range = max(all_z) - min(all_z)
    if z_range < 1e-6:
        z_range = 1.0

    # Rule A
    horiz = [f for f in faces if f.surface_type=="plane" and abs(f.normal[2])>0.85]
    if horiz:
        lo = min(f.centroid[2] for f in horiz)
        tol = max(z_range*0.05, 0.5)
        bp = [f for f in horiz if f.centroid[2] < lo+tol]
        ta = sum(f.area for f in bp)
        if ta > 0:
            cx = sum(f.centroid[0]*f.area for f in bp)/ta
            cy = sum(f.centroid[1]*f.area for f in bp)/ta
            cz = sum(f.centroid[2]*f.area for f in bp)/ta
            ma = sum(f.area for f in faces)
            conf = min(0.95, 0.4 + (ta/ma)*2.5) if ma else 0.5
            proposals.append(Proposal("rule_a","Lowest support plane",
                (cx,cy,cz),(0,0,1),round(conf,2),[f.id for f in bp]))

    # Rule B
    ap = [f for f in faces if f.surface_type=="plane"]
    if len(ap)>=2:
        ap.sort(key=lambda f:f.bbox_min[2])
        lo = ap[0].bbox_min[2]; ct = max(z_range*0.10,1.0)
        cl = [f for f in ap if f.bbox_min[2]<lo+ct]
        if len(cl)>=2:
            ta = sum(f.area for f in cl)
            if ta>0:
                cx = sum(f.centroid[0]*f.area for f in cl)/ta
                cy = sum(f.centroid[1]*f.area for f in cl)/ta
                cz = min(f.centroid[2] for f in cl)
                proposals.append(Proposal("rule_b","Stable contact region",
                    (cx,cy,cz),(0,0,1),round(min(0.75,0.35+len(cl)*0.05),2),
                    [f.id for f in cl]))

    # Rule C
    if len(faces)>=4:
        wp = [(f.centroid,f.area) for f in faces]
        tw = sum(a for _,a in wp)
        if tw>0:
            mx=sum(c[0]*a for c,a in wp)/tw
            my=sum(c[1]*a for c,a in wp)/tw
            mz=sum(c[2]*a for c,a in wp)/tw
            cov=[[0.0]*3 for _ in range(3)]
            for c,a in wp:
                dc=(c[0]-mx,c[1]-my,c[2]-mz)
                for i in range(3):
                    for j in range(3): cov[i][j]+=a*dc[i]*dc[j]
            for i in range(3):
                for j in range(3): cov[i][j]/=tw
            v=[1.0,0.3,0.1]
            for _ in range(200):
                nv=[sum(cov[i][j]*v[j] for j in range(3)) for i in range(3)]
                nl=math.sqrt(sum(x*x for x in nv))
                if nl<1e-15: break
                v=[x/nl for x in nv]
            pa=tuple(v)
            l1=sum(cov[i][j]*v[i]*v[j] for i in range(3) for j in range(3))
            tr=cov[0][0]+cov[1][1]+cov[2][2]
            if tr>1e-6 and l1/tr>0.55:
                dots=[(_dot(f.centroid,pa),f) for f in faces]
                dmin=min(d for d,_ in dots); dmax=max(d for d,_ in dots)
                dr=dmax-dmin
                if dr>1e-6:
                    bf=[f for d,f in dots if d<dmin+dr*0.12]
                    if bf:
                        ta=sum(f.area for f in bf)
                        if ta>0:
                            cx=sum(f.centroid[0]*f.area for f in bf)/ta
                            cy=sum(f.centroid[1]*f.area for f in bf)/ta
                            cz=sum(f.centroid[2]*f.area for f in bf)/ta
                            pa2=pa if pa[2]>=0 else tuple(-x for x in pa)
                            proposals.append(Proposal("rule_c","PCA axis + bottom end",
                                (cx,cy,cz),pa2,round(min(0.80,0.3+l1/tr),2),
                                [f.id for f in bf]))

    proposals.sort(key=lambda p:-p.confidence)
    return proposals


# ---------------------------------------------------------------------------
# Method 2: Joint feature detection
# ---------------------------------------------------------------------------

def find_joint_features(faces: List[FaceInfo]) -> List[JointFeature]:
    features: List[JointFeature] = []
    cyls=[f for f in faces if f.surface_type=="cylinder" and f.axis_origin and f.axis_direction]
    used=set()
    for i,c1 in enumerate(cyls):
        if i in used: continue
        grp=[c1]; used.add(i)
        for j,c2 in enumerate(cyls):
            if j in used: continue
            if abs(_dot(c1.axis_direction,c2.axis_direction))<0.97: continue
            diff=_sub(c2.axis_origin,c1.axis_origin); dl=_length(diff)
            if dl>1e-6 and _length(_cross(diff,c1.axis_direction))/dl>0.15: continue
            grp.append(c2); used.add(j)
        if len(grp)>=2:
            ao=tuple(sum(f.axis_origin[k] for f in grp)/len(grp) for k in range(3))
            ad=_normalize(tuple(sum(f.axis_direction[k] for f in grp)/len(grp) for k in range(3)))
            ar=sum(f.radius for f in grp if f.radius)/max(1,sum(1 for f in grp if f.radius))
            features.append(JointFeature("revolute",ao,ad,[f.id for f in grp],round(ar,3)))

    planes=[f for f in faces if f.surface_type=="plane" and f.area>1.0]
    used_p=set()
    for i,p1 in enumerate(planes):
        if i in used_p: continue
        grp=[p1]; used_p.add(i)
        for j,p2 in enumerate(planes):
            if j in used_p: continue
            if abs(_dot(p1.normal,p2.normal))>0.98: grp.append(p2); used_p.add(j)
        if len(grp)>=4:
            ao=tuple(sum(f.centroid[k] for f in grp)/len(grp) for k in range(3))
            n=_normalize(grp[0].normal)
            u=_normalize(_cross(n,(0,0,1))) if abs(n[2])<0.9 else _normalize(_cross(n,(1,0,0)))
            vv=_cross(n,u)
            pts=[(_dot(f.centroid,u),_dot(f.centroid,vv)) for f in grp]
            us=max(p[0] for p in pts)-min(p[0] for p in pts)
            vs=max(p[1] for p in pts)-min(p[1] for p in pts)
            features.append(JointFeature("prismatic",ao,u if us>=vs else vv,[f.id for f in grp]))
    return features


# ---------------------------------------------------------------------------
# Transform & export
# ---------------------------------------------------------------------------

def build_transform(origin, z_axis, x_rot_deg=0):
    """Build gp_Trsf: translate origin->world-O, rotate z_axis->+Z,
    then rotate *x_rot_deg* degrees around Z (adjusts X/Y orientation)."""
    zf = _normalize(z_axis)
    zt = (0.0, 0.0, 1.0)
    d = _dot(zf, zt)

    rot = gp_Trsf()
    if d < -0.99999:
        perp = _normalize(_cross(zf, (1,0,0))) if abs(zf[0])<0.9 \
               else _normalize(_cross(zf, (0,1,0)))
        rot.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(*perp)), math.pi)
    elif d < 0.99999:
        ax = _normalize(_cross(zf, zt))
        rot.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(*ax)),
                        math.acos(max(-1.0, min(1.0, d))))

    if x_rot_deg:
        rz = gp_Trsf()
        rz.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)),
                       math.radians(x_rot_deg))
        rot = rz.Multiplied(rot)

    trans = gp_Trsf()
    trans.SetTranslation(gp_Vec(-origin[0], -origin[1], -origin[2]))
    return rot.Multiplied(trans)


def trsf_to_record(trsf, origin, z_axis, x_rot_deg, unit, source_file):
    """Extract a JSON-serialisable transform record from the gp_Trsf."""
    R = [[trsf.Value(i+1, j+1) for j in range(3)] for i in range(3)]
    T = [trsf.Value(i+1, 4) for i in range(3)]
    euler = _rot_matrix_to_euler_xyz(R)
    x_axis = [R[0][0], R[1][0], R[2][0]]
    y_axis = [R[0][1], R[1][1], R[2][1]]
    z_out  = [R[0][2], R[1][2], R[2][2]]
    return {
        "source_file": os.path.basename(source_file),
        "unit": unit,
        "selected_origin": [round(v, 6) for v in origin],
        "selected_z_axis": [round(v, 6) for v in z_axis],
        "x_rotation_deg": x_rot_deg,
        "transform": {
            "translation": [round(v, 6) for v in T],
            "rotation_matrix_3x3": [[round(c, 8) for c in row] for row in R],
            "rotation_euler_xyz_deg": [round(v, 4) for v in euler],
        },
        "canonical_axes": {
            "x": [round(v, 6) for v in x_axis],
            "y": [round(v, 6) for v in y_axis],
            "z": [round(v, 6) for v in z_out],
        },
    }


def _detect_step_schema(path: str) -> str:
    """Read FILE_SCHEMA from a STEP file header to match on export."""
    with open(path, "r", errors="replace") as f:
        head = f.read(4096)
    if "AP242" in head:
        return "AP242DIS"
    if "AP203" in head:
        return "AP203"
    return "AP214"


def _with_suppressed_stdout(fn):
    """Run an OCC call while suppressing C-level stdout chatter."""
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_fd = os.dup(1)
    os.dup2(devnull_fd, 1)
    try:
        return fn()
    finally:
        os.dup2(saved_fd, 1)
        os.close(devnull_fd)
        os.close(saved_fd)


_STEP_REAL_RE = re.compile(
    r"(?<![#A-Za-z0-9_])[-+]?(?:(?:\d+\.\d*|\.\d+)(?:[Ee][+-]?\d+)?|\d+[Ee][+-]?\d+)"
)


def _detect_step_decimal_places(path: str, default: int = 10) -> int:
    """Return the source STEP file's maximum real-literal decimal places."""
    max_places = 0
    with open(path, "r", errors="replace") as f:
        text = f.read()

    def scan_chunk(chunk):
        nonlocal max_places
        for match in _STEP_REAL_RE.finditer(chunk):
            mantissa = re.split("[Ee]", match.group(0), 1)[0]
            if "." in mantissa:
                max_places = max(max_places, len(mantissa.split(".", 1)[1]))

    _for_step_chunks_outside_strings(text, scan_chunk)
    return max_places or default


def _for_step_chunks_outside_strings(text, callback):
    """Call callback on STEP text chunks, skipping quoted string literals."""
    i = 0
    n = len(text)
    while i < n:
        quote = text.find("'", i)
        if quote < 0:
            callback(text[i:])
            break
        if quote > i:
            callback(text[i:quote])

        i = quote + 1
        while i < n:
            if text[i] == "'":
                if i + 1 < n and text[i + 1] == "'":
                    i += 2
                    continue
                i += 1
                break
            i += 1


def _format_step_real(token: str, decimal_places: int) -> str:
    try:
        value = Decimal(token)
    except InvalidOperation:
        return token

    quantum = Decimal(1).scaleb(-decimal_places)
    rounded = value.quantize(quantum, rounding=ROUND_HALF_UP)
    text = format(rounded, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if text in ("", "-0", "+0"):
        text = "0"
    if "." not in text:
        text += "."
    return text


def _normalize_step_real_precision(path: str, decimal_places: int):
    """Keep exported STEP real precision close to the source file."""
    with open(path, "r", errors="replace") as f:
        text = f.read()

    out = []
    i = 0
    n = len(text)
    while i < n:
        quote = text.find("'", i)
        end = n if quote < 0 else quote
        out.append(_STEP_REAL_RE.sub(
            lambda m: _format_step_real(m.group(0), decimal_places),
            text[i:end]))
        if quote < 0:
            break

        start = quote
        i = quote + 1
        while i < n:
            if text[i] == "'":
                if i + 1 < n and text[i + 1] == "'":
                    i += 2
                    continue
                i += 1
                break
            i += 1
        out.append(text[start:i])

    with open(path, "w") as f:
        f.write("".join(out))


def _export_transformed_step_xcaf(step_path, trsf, output_path, schema):
    """Transform an XCAF STEP document so product/component names survive."""
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    reader.SetLayerMode(True)
    reader.SetPropsMode(True)

    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read STEP file: {step_path}")
    if not reader.Transfer(doc):
        return False

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free_shapes = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_shapes)
    root_loc = TopLoc_Location(trsf)

    def component_children(label):
        ref_label = TDF_Label()
        if XCAFDoc_ShapeTool.GetReferredShape_s(label, ref_label):
            label = ref_label
        children = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(label, children)
        return children

    def transform_leaf_components(label, parent_loc):
        children = component_children(label)
        if not children.Length():
            return False

        transformed_any = False
        for j in range(1, children.Length() + 1):
            component = children.Value(j)
            old_loc = XCAFDoc_ShapeTool.GetLocation_s(component)
            child_parent_loc = parent_loc.Multiplied(old_loc)
            if transform_leaf_components(component, child_parent_loc):
                transformed_any = True
                continue

            new_loc = (
                parent_loc.Inverted()
                .Multiplied(root_loc)
                .Multiplied(parent_loc)
                .Multiplied(old_loc)
            )
            ref_label = TDF_Label()
            shape_tool.SetLocation(component, new_loc, ref_label)
            transformed_any = True
        return transformed_any

    for i in range(1, free_shapes.Length() + 1):
        label = free_shapes.Value(i)
        if not transform_leaf_components(label, TopLoc_Location()):
            shape = shape_tool.GetShape_s(label)
            builder = BRepBuilderAPI_Transform(shape, trsf, True)
            builder.Build()
            shape_tool.SetShape(label, builder.Shape())

    shape_tool.UpdateAssemblies()

    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.SetColorMode(True)
    writer.SetLayerMode(True)
    writer.SetPropsMode(True)
    Interface_Static.SetCVal_s("write.step.schema", schema)

    if not _with_suppressed_stdout(lambda: writer.Transfer(doc, STEPControl_AsIs)):
        return False
    return _with_suppressed_stdout(
        lambda: writer.Write(str(output_path)) == IFSelect_RetDone)


def export_transformed_step(step_path, origin, z_axis, output_path,
                            x_rot_deg=0, unit="mm"):
    """Transform via OCC (correct for assemblies). Returns (ok, record)."""
    trsf = build_transform(origin, z_axis, x_rot_deg)
    schema = _detect_step_schema(step_path)
    decimal_places = _detect_step_decimal_places(step_path)

    ok = _export_transformed_step_xcaf(step_path, trsf, output_path, schema)
    if not ok:
        shape = load_step(step_path)
        builder = BRepBuilderAPI_Transform(shape, trsf, True)
        builder.Build()
        new_shape = builder.Shape()

        writer = STEPControl_Writer()
        Interface_Static.SetCVal_s("write.step.schema", schema)
        _with_suppressed_stdout(
            lambda: writer.Transfer(new_shape, STEPControl_AsIs))
        ok = _with_suppressed_stdout(
            lambda: writer.Write(str(output_path)) == IFSelect_RetDone)

    if ok:
        _normalize_step_real_precision(str(output_path), decimal_places)

    record = trsf_to_record(trsf, origin, z_axis, x_rot_deg, unit,
                            str(output_path))
    return ok, record


# ---------------------------------------------------------------------------
# CLI output helpers
# ---------------------------------------------------------------------------

def print_results(filepath, faces, proposals, features, unit="mm"):
    name = os.path.basename(filepath)
    np_ = sum(1 for f in faces if f.surface_type=="plane")
    nc  = sum(1 for f in faces if f.surface_type=="cylinder")
    print(f"\n{'='*60}")
    print(f"  {name}  (unit: {unit})")
    print(f"  Faces: {len(faces)} total  ({np_} plane, {nc} cyl, "
          f"{len(faces)-np_-nc} other)")
    print(f"{'='*60}")
    if proposals:
        print(f"\n  --- Bottom Center Proposals ---")
        for i,p in enumerate(proposals):
            tag=" *BEST*" if i==0 else ""
            print(f"\n  [{p.method}] {p.label}{tag}")
            print(f"    Center : ({p.center[0]:>10.3f}, {p.center[1]:>10.3f}, {p.center[2]:>10.3f}) {unit}")
            print(f"    Z-axis : ({p.z_axis[0]:>7.4f}, {p.z_axis[1]:>7.4f}, {p.z_axis[2]:>7.4f})")
            print(f"    Confidence : {p.confidence:.0%}")
    else:
        print("\n  No auto proposals. Use --ui for manual selection.")
    if features:
        print(f"\n  --- Joint Features ---")
        for jf in features:
            print(f"\n  [{jf.type}]")
            print(f"    Axis origin: ({jf.axis_origin[0]:.1f}, {jf.axis_origin[1]:.1f}, {jf.axis_origin[2]:.1f})")
            if jf.radius: print(f"    Radius: {jf.radius:.3f}")
    print()


# ---------------------------------------------------------------------------
# Web UI
# ---------------------------------------------------------------------------

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"><title>Bottom Center Finder</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
     overflow:hidden;background:#1a1a2e;color:#e0e0e0}
#viewer{position:absolute;top:0;left:0}
#panel{position:absolute;top:0;right:0;width:370px;height:100vh;
       background:rgba(26,26,46,0.97);border-left:1px solid #333;
       overflow-y:auto;padding:16px 18px;z-index:10}
#panel h2{font-size:17px;color:#4fc3f7;margin-bottom:12px;font-weight:600}
#panel h3{font-size:13px;color:#999;text-transform:uppercase;letter-spacing:.5px;
          margin:16px 0 8px;padding-top:12px;border-top:1px solid #2a2a4a}
#panel h3:first-of-type{border-top:none;padding-top:0}
.card{padding:10px 12px;margin:4px 0;border-radius:6px;cursor:pointer;
      background:rgba(255,255,255,0.04);border:1px solid transparent;transition:.15s}
.card:hover{background:rgba(255,255,255,0.09);border-color:#444}
.card.selected{background:rgba(76,175,80,0.15);border-color:#4caf50}
.card-title{font-size:13px;font-weight:600;margin-bottom:3px}
.card-sub{font-size:11px;color:#888;line-height:1.5}
.info-box{padding:10px 12px;background:rgba(255,255,255,0.03);border-radius:6px;
          margin:6px 0;font-size:12px;line-height:1.8}
.info-box .lbl{color:#777;display:inline-block;width:55px}
.info-box .val{color:#e0e0e0;font-family:'SF Mono',Consolas,monospace;font-size:11px}
button{background:#4fc3f7;color:#111;border:none;padding:7px 16px;border-radius:5px;
       cursor:pointer;font-size:13px;font-weight:500;margin:3px;transition:.15s}
button:hover{background:#81d4fa}
button.warn{background:#ff7043} button.warn:hover{background:#ff8a65}
button.green{background:#66bb6a;color:#fff} button.green:hover{background:#81c784}
button:disabled{opacity:.4;cursor:default}
.conf-bar{display:inline-block;height:6px;border-radius:3px;margin-left:6px;vertical-align:middle}
#status{position:fixed;top:12px;left:12px;background:rgba(0,0,0,.7);color:#4fc3f7;
        padding:6px 14px;border-radius:20px;font-size:12px;z-index:20;pointer-events:none}
#help-hint{position:fixed;bottom:12px;left:12px;color:#555;font-size:11px;z-index:20;pointer-events:none}
.btn-row{display:flex;gap:4px;margin:4px 0;flex-wrap:wrap}
.btn-row button{flex:1;min-width:0}
</style>
</head>
<body>
<canvas id="viewer"></canvas>
<div id="status">Loading model...</div>
<div id="help-hint">Click face to select | Scroll to zoom | Drag to orbit</div>
<div id="panel">
  <h2>Bottom Center Finder</h2>
  <div id="filename" style="font-size:12px;color:#888;margin-bottom:2px"></div>
  <div id="face-count" style="font-size:11px;color:#666;margin-bottom:8px"></div>

  <h3>Auto Proposals</h3>
  <div id="proposals"></div>

  <h3>Selected Face</h3>
  <div id="face-info" class="info-box" style="color:#666">Click any face on the model</div>

  <div id="sel-controls" style="display:none">
    <h3>Canonical Frame</h3>
    <div class="info-box">
      <div><span class="lbl">Origin</span><span class="val" id="v-origin">-</span></div>
      <div><span class="lbl">Z-axis</span><span class="val" id="v-zaxis">-</span></div>
      <div><span class="lbl">Dir</span><span class="val" id="v-dir">-</span></div>
      <div><span class="lbl">X rot</span><span class="val" id="v-xrot">0</span></div>
    </div>
    <div class="btn-row">
      <button class="warn" onclick="flipZ()">Flip Z</button>
      <button class="warn" onclick="rotateX()">Rotate X +90</button>
    </div>
    <div class="btn-row">
      <button onclick="dropToBottom()">Drop to bottom</button>
    </div>
  </div>

  <div style="margin-top:20px;padding-top:14px;border-top:1px solid #2a2a4a">
    <button onclick="doConfirm()" id="btn-confirm"
            style="width:100%;padding:11px;font-size:14px;font-weight:700;margin-bottom:6px">
      Confirm (move to origin)
    </button>
    <button class="green" onclick="doExport()" id="btn-export" disabled
            style="width:100%;padding:11px;font-size:14px;font-weight:700">
      Export STEP + Record
    </button>
    <div id="export-status" style="font-size:11px;color:#888;margin-top:6px;text-align:center"></div>
  </div>
</div>

<script type="importmap">
{"imports":{
  "three":"https://cdn.jsdelivr.net/npm/three@0.168.0/build/three.module.js",
  "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.168.0/examples/jsm/"
}}
</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

let DATA=null;
const faceMeshes=[];
let hoveredMesh=null, selectedMesh=null, modelGroup=null;
let curOrigin=null, curZAxis=[0,0,1], curXRotDeg=0, confirmed=false;
const markers=[];

const C_DEFAULT=0xbbbbbb, C_SELECTED=0x4caf50, C_PROPOSAL=0xffab40;

const canvas=document.getElementById('viewer');
const PW=370;
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x1a1a2e);
const camera=new THREE.PerspectiveCamera(45,(innerWidth-PW)/innerHeight,0.01,1e6);
camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
renderer.setSize(innerWidth-PW,innerHeight); renderer.setPixelRatio(devicePixelRatio);
const controls=new OrbitControls(camera,canvas);
controls.enableDamping=true; controls.dampingFactor=0.08;

scene.add(new THREE.AmbientLight(0x404040,2.0));
const dl1=new THREE.DirectionalLight(0xffffff,1.2); dl1.position.set(1,.5,1.5); scene.add(dl1);
const dl2=new THREE.DirectionalLight(0xffffff,.6); dl2.position.set(-1,-1,-.5); scene.add(dl2);

const rc=new THREE.Raycaster(); const mouse=new THREE.Vector2();

fetch('/api/model-data').then(r=>r.json()).then(data=>{
    DATA=data; buildScene(data); buildUI(data);
    document.getElementById('status').textContent=
        data.filename+' \u2014 '+data.faces.length+' faces ('+data.unit+')';
}).catch(e=>{ document.getElementById('status').textContent='Error: '+e; });

function buildScene(data){
    const g=new THREE.Group();
    data.meshes.forEach(m=>{
        if(!m.positions.length)return;
        const geo=new THREE.BufferGeometry();
        geo.setAttribute('position',new THREE.Float32BufferAttribute(m.positions,3));
        geo.setAttribute('normal',new THREE.Float32BufferAttribute(m.normals,3));
        const mat=new THREE.MeshPhongMaterial({color:C_DEFAULT,side:THREE.DoubleSide,
            transparent:true,opacity:.92,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1});
        const mesh=new THREE.Mesh(geo,mat);
        mesh.userData={faceId:m.face_id,info:data.faces[m.face_id]};
        g.add(mesh); faceMeshes.push(mesh);
    });
    data.meshes.forEach(m=>{
        if(!m.positions.length)return;
        const geo=new THREE.BufferGeometry();
        geo.setAttribute('position',new THREE.Float32BufferAttribute(m.positions,3));
        g.add(new THREE.Mesh(geo,new THREE.MeshBasicMaterial({color:0,wireframe:true,transparent:true,opacity:.08})));
    });
    scene.add(g); modelGroup=g;

    const bb=data.bbox;
    const cx=(bb.min[0]+bb.max[0])/2,cy=(bb.min[1]+bb.max[1])/2,cz=(bb.min[2]+bb.max[2])/2;
    const sx=bb.max[0]-bb.min[0],sy=bb.max[1]-bb.min[1],sz=bb.max[2]-bb.min[2];
    const md=Math.max(sx,sy,sz)||100;
    camera.position.set(cx+md,cy-md*.8,cz+md*.7);
    controls.target.set(cx,cy,cz);
    camera.near=md*.001; camera.far=md*100; camera.updateProjectionMatrix(); controls.update();

    const grid=new THREE.GridHelper(md*2,20,0x333355,0x222244);
    grid.rotation.x=Math.PI/2; grid.position.set(cx,cy,bb.min[2]); scene.add(grid);
    const ax=new THREE.AxesHelper(md*.25); ax.position.set(bb.min[0],bb.min[1],bb.min[2]); scene.add(ax);

    if(data.proposals.length) highlightProposal(data.proposals[0]);
}

function buildUI(data){
    const u=data.unit;
    document.getElementById('filename').textContent=data.filename;
    document.getElementById('face-count').textContent=
        data.faces.length+' faces ('+
        data.faces.filter(f=>f.surface_type==='plane').length+' plane, '+
        data.faces.filter(f=>f.surface_type==='cylinder').length+' cyl) \u2014 unit: '+u;

    const pdiv=document.getElementById('proposals');
    if(!data.proposals.length){
        pdiv.innerHTML='<div style="color:#666;font-size:12px">None \u2014 click a face</div>';
    } else {
        data.proposals.forEach(p=>{
            const d=document.createElement('div'); d.className='card';
            const cw=Math.round(p.confidence*60);
            const cc=p.confidence>.7?'#4caf50':p.confidence>.4?'#ffab40':'#ef5350';
            d.innerHTML='<div class="card-title">'+p.label+'</div>'+
                '<div class="card-sub">Center: ('+p.center.map(v=>v.toFixed(1)).join(', ')+') '+u+'</div>'+
                '<div class="card-sub">Conf: '+(p.confidence*100).toFixed(0)+'%'+
                '<span class="conf-bar" style="width:'+cw+'px;background:'+cc+'"></span></div>';
            d.onclick=()=>selectProposal(p,d);
            pdiv.appendChild(d);
        });
    }
}

/* ── helpers ── */
function resetColors(){ faceMeshes.forEach(m=>{m.material.color.setHex(C_DEFAULT);m.material.opacity=.92;m.material.emissive.setHex(0)}); }
function clearMarkers(){ markers.forEach(m=>scene.remove(m)); markers.length=0; }
function addMarker(o){ markers.push(o); scene.add(o); }
function maxDim(){ const b=DATA.bbox; return Math.max(b.max[0]-b.min[0],b.max[1]-b.min[1],b.max[2]-b.min[2])||100; }

/* Compute model-space X/Y/Z directions for the canonical frame */
function canonicalAxesModel(zAxis, xRotDeg){
    const zFrom=new THREE.Vector3(...zAxis).normalize();
    const zTo=new THREE.Vector3(0,0,1);
    const q=new THREE.Quaternion().setFromUnitVectors(zFrom,zTo);
    if(xRotDeg){ q.premultiply(new THREE.Quaternion().setFromAxisAngle(zTo,xRotDeg*Math.PI/180)); }
    const qi=q.clone().invert();
    return {
        x: new THREE.Vector3(1,0,0).applyQuaternion(qi),
        y: new THREE.Vector3(0,1,0).applyQuaternion(qi),
        z: zFrom.clone(),
    };
}

/* Show origin sphere + X/Y/Z arrows at a point in model space */
function showFrameAxes(c, zAxis, xRotDeg){
    clearMarkers();
    const md=maxDim();
    const pos=new THREE.Vector3(...c);
    // origin sphere
    const sg=new THREE.SphereGeometry(md*.012,20,20);
    const sph=new THREE.Mesh(sg,new THREE.MeshBasicMaterial({color:0xffffff,depthTest:false}));
    sph.renderOrder=999; sph.position.copy(pos); addMarker(sph);
    // axes
    const ax=canonicalAxesModel(zAxis,xRotDeg);
    addMarker(new THREE.ArrowHelper(ax.x, pos, md*.22, 0xff1744, md*.016, md*.01));  // X red
    addMarker(new THREE.ArrowHelper(ax.z, pos, md*.25, 0x448aff, md*.018, md*.012)); // Z blue
}

function highlightProposal(p){
    resetColors(); confirmed=false; resetVisualTransform();
    document.getElementById('btn-export').disabled=true;
    p.face_ids.forEach(id=>{ const m=faceMeshes.find(m=>m.userData.faceId===id); if(m){m.material.color.setHex(C_PROPOSAL);m.material.opacity=1;} });
    curOrigin=p.center; curZAxis=[...p.z_axis]; curXRotDeg=0;
    showFrameAxes(p.center,p.z_axis,curXRotDeg); updateFrameUI();
}
function selectProposal(p,el){
    document.querySelectorAll('#proposals .card').forEach(d=>d.classList.remove('selected'));
    el.classList.add('selected'); selectedMesh=null; highlightProposal(p);
}
function updateFrameUI(){
    if(!curOrigin)return;
    document.getElementById('sel-controls').style.display='';
    const u=DATA.unit;
    const fmt=v=>v.map(x=>x.toFixed(3)).join(', ');
    document.getElementById('v-origin').textContent='('+fmt(curOrigin)+') '+u;
    document.getElementById('v-zaxis').textContent='('+fmt(curZAxis)+')';
    document.getElementById('v-dir').textContent=curZAxis[2]>.01?'Up (+Z)':curZAxis[2]<-.01?'Down (-Z)':'Lateral';
    document.getElementById('v-xrot').textContent=curXRotDeg+'\u00B0';
}

/* ── mouse ── */
canvas.addEventListener('mousemove',e=>{
    const r=canvas.getBoundingClientRect();
    mouse.x=((e.clientX-r.left)/r.width)*2-1;
    mouse.y=-((e.clientY-r.top)/r.height)*2+1;
    rc.setFromCamera(mouse,camera);
    const hits=rc.intersectObjects(faceMeshes);
    if(hoveredMesh&&hoveredMesh!==selectedMesh) hoveredMesh.material.emissive.setHex(0);
    if(hits.length){ hoveredMesh=hits[0].object; if(hoveredMesh!==selectedMesh) hoveredMesh.material.emissive.setHex(0x111133); canvas.style.cursor='pointer'; }
    else{ hoveredMesh=null; canvas.style.cursor='default'; }
});

canvas.addEventListener('click',e=>{
    if(!hoveredMesh)return;
    const info=hoveredMesh.userData.info; if(!info)return;
    confirmed=false; document.getElementById('btn-export').disabled=true;
    if(selectedMesh){selectedMesh.material.color.setHex(C_DEFAULT);selectedMesh.material.opacity=.92;}
    selectedMesh=hoveredMesh; selectedMesh.material.color.setHex(C_SELECTED); selectedMesh.material.opacity=1;
    const u=DATA.unit;
    document.getElementById('face-info').innerHTML=
        '<div><span class="lbl">ID</span><span class="val">'+info.id+'</span></div>'+
        '<div><span class="lbl">Type</span><span class="val">'+info.surface_type+'</span></div>'+
        '<div><span class="lbl">Area</span><span class="val">'+info.area.toFixed(2)+' '+u+'\u00B2</span></div>'+
        '<div><span class="lbl">Center</span><span class="val">('+info.centroid.map(v=>v.toFixed(2)).join(', ')+') '+u+'</span></div>'+
        '<div><span class="lbl">Normal</span><span class="val">('+info.normal.map(v=>v.toFixed(4)).join(', ')+')</span></div>';
    curOrigin=info.centroid; curZAxis=[...info.normal]; curXRotDeg=0;
    if(curZAxis[2]<0) curZAxis=curZAxis.map(x=>-x);
    showFrameAxes(curOrigin,curZAxis,curXRotDeg); updateFrameUI();
    document.querySelectorAll('#proposals .card').forEach(d=>d.classList.remove('selected'));
});

/* ── actions ── */
window.flipZ=function(){
    curZAxis=curZAxis.map(x=>-x);
    updateFrameUI();
    if(confirmed){ applyVisualTransform(curOrigin,curZAxis); }
    else if(curOrigin){ showFrameAxes(curOrigin,curZAxis,curXRotDeg); }
};
window.rotateX=function(){
    curXRotDeg=(curXRotDeg+90)%360;
    updateFrameUI();
    // update arrows to reflect new X orientation (mesh stays)
    if(confirmed){ showWorldAxes(); }
    else if(curOrigin){ showFrameAxes(curOrigin,curZAxis,curXRotDeg); }
};

function resetVisualTransform(){
    if(!modelGroup)return;
    modelGroup.quaternion.identity();
    modelGroup.position.set(0,0,0);
}

window.doConfirm=function(){
    if(!curOrigin){alert('Select a face or proposal first');return;}
    confirmed=true;
    applyVisualTransform(curOrigin,curZAxis);
    document.getElementById('btn-export').disabled=false;
    document.getElementById('export-status').textContent='';
    document.getElementById('status').textContent=DATA.filename+' \u2014 confirmed at origin';
};

window.doExport=function(){
    if(!confirmed||!curOrigin)return;
    const btn=document.getElementById('btn-export');
    const st=document.getElementById('export-status');
    btn.disabled=true; st.textContent='Exporting STEP...'; st.style.color='#888';

    const body=JSON.stringify({origin:curOrigin,z_axis:curZAxis,x_rot_deg:curXRotDeg});

    (async()=>{
    try{
        // 1) STEP file
        const r1=await fetch('/api/export-step',{method:'POST',headers:{'Content-Type':'application/json'},body});
        if(!r1.ok){ const t=await r1.text(); throw new Error('STEP export: '+r1.status+' '+t.slice(0,200)); }
        dl(await r1.blob(), DATA.filename.replace(/\.step$/i,'_centered.step'));
        st.textContent='STEP downloaded. Fetching record...';
        // 2) Transform record JSON
        const r2=await fetch('/api/transform-record',{method:'POST',headers:{'Content-Type':'application/json'},body});
        if(!r2.ok){ const t=await r2.text(); throw new Error('Record: '+r2.status+' '+t.slice(0,200)); }
        dl(await r2.blob(), DATA.filename.replace(/\.step$/i,'_transform.json'));
        st.textContent='Done \u2014 STEP + record downloaded.'; st.style.color='#4caf50';
        btn.disabled=false;
    }catch(err){
        st.textContent='Export failed: '+err.message; st.style.color='#ef5350'; btn.disabled=false;
    }
    })();
};

function dl(blob,name){ const a=document.createElement('a'); a.href=URL.createObjectURL(blob); a.download=name; a.click(); URL.revokeObjectURL(a.href); }

function applyVisualTransform(origin,zAxis){
    if(!modelGroup)return;
    const zFrom=new THREE.Vector3(...zAxis).normalize();
    const zTo=new THREE.Vector3(0,0,1);
    const quat=new THREE.Quaternion().setFromUnitVectors(zFrom,zTo);
    const ro=new THREE.Vector3(...origin).applyQuaternion(quat);
    modelGroup.quaternion.copy(quat);
    modelGroup.position.set(-ro.x,-ro.y,-ro.z);
    controls.target.set(0,0,0);
    showWorldAxes();
}

/* Show X/Y/Z arrows at world origin after confirm.
   The visual only applies Z-align (no xRot on mesh),
   so the canonical X in visual-space = Rz(-xRot)*(1,0,0). */
function showWorldAxes(){
    clearMarkers();
    const md=maxDim(); const o=new THREE.Vector3(0,0,0);
    // sphere
    const sg=new THREE.SphereGeometry(md*.012,20,20);
    const sph=new THREE.Mesh(sg,new THREE.MeshBasicMaterial({color:0xffffff,depthTest:false}));
    sph.renderOrder=999; addMarker(sph);
    // Z always up
    addMarker(new THREE.ArrowHelper(new THREE.Vector3(0,0,1),o,md*.25,0x448aff,md*.018,md*.012));
    // X: undo the xRot that export will apply (since mesh isn't rotated)
    const xDir=new THREE.Vector3(1,0,0);
    if(curXRotDeg){
        const q=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1),-curXRotDeg*Math.PI/180);
        xDir.applyQuaternion(q);
    }
    addMarker(new THREE.ArrowHelper(xDir,o,md*.22,0xff1744,md*.016,md*.01));
}

/* Drop origin Z to the lowest mesh vertex along curZAxis direction.
   Works for flat planes AND curved bottoms (circles, cylinders). */
window.dropToBottom=function(){
    if(!curOrigin||!DATA)return;
    const zx=curZAxis[0], zy=curZAxis[1], zz=curZAxis[2];
    let minProj=Infinity;
    DATA.meshes.forEach(m=>{
        const p=m.positions;
        for(let i=0;i<p.length;i+=3){
            const proj=p[i]*zx+p[i+1]*zy+p[i+2]*zz;
            if(proj<minProj) minProj=proj;
        }
    });
    if(!isFinite(minProj))return;
    const curProj=curOrigin[0]*zx+curOrigin[1]*zy+curOrigin[2]*zz;
    const shift=minProj-curProj;
    curOrigin=[curOrigin[0]+shift*zx, curOrigin[1]+shift*zy, curOrigin[2]+shift*zz];
    updateFrameUI();
    if(confirmed){ applyVisualTransform(curOrigin,curZAxis); }
    else{ showFrameAxes(curOrigin,curZAxis,curXRotDeg); }
};

addEventListener('resize',()=>{ camera.aspect=(innerWidth-PW)/innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth-PW,innerHeight); });
(function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene,camera); })();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):
    _model_data = None
    _shape = None
    _step_path = None
    _unit = "mm"

    def do_GET(self):
        if self.path == "/":
            self._respond(200, "text/html", HTML_PAGE.encode())
        elif self.path == "/api/model-data":
            self._respond(200, "application/json",
                          json.dumps(self._model_data).encode())
        else:
            self._respond(404, "text/plain", b"Not found")

    def do_POST(self):
        try:
            if self.path == "/api/export-step":
                self._handle_export_step()
            elif self.path == "/api/transform-record":
                self._handle_transform_record()
            else:
                self._respond(404, "text/plain", b"Not found")
        except Exception:
            msg = traceback.format_exc()
            print(f"  POST {self.path} ERROR:\n{msg}", file=sys.stderr)
            self._respond(500, "text/plain", msg.encode())

    def _read_body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n))

    def _handle_export_step(self):
        body = self._read_body()
        origin = tuple(body["origin"])
        z_axis = tuple(body["z_axis"])
        x_rot_deg = int(body.get("x_rot_deg", 0))

        fd, tmp = tempfile.mkstemp(suffix=".step")
        os.close(fd)
        try:
            ok, _ = export_transformed_step(
                self._step_path, origin, z_axis, tmp,
                x_rot_deg=x_rot_deg, unit=self._unit)
            if not ok:
                self._respond(500, "text/plain", b"STEP write failed")
                return
            with open(tmp, "rb") as f:
                data = f.read()
            stem = Path(self._step_path).stem
            fname = f"{stem}_centered.step"
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Disposition",
                             f'attachment; filename="{fname}"')
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            print(f"  Exported {fname} ({len(data)} bytes)")
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def _handle_transform_record(self):
        body = self._read_body()
        origin = tuple(body["origin"])
        z_axis = tuple(body["z_axis"])
        x_rot_deg = int(body.get("x_rot_deg", 0))
        trsf = build_transform(origin, z_axis, x_rot_deg)
        record = trsf_to_record(trsf, origin, z_axis, x_rot_deg,
                                self._unit, self._step_path)
        payload = json.dumps(record, indent=2).encode()
        stem = Path(self._step_path).stem
        fname = f"{stem}_transform.json"
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Disposition",
                         f'attachment; filename="{fname}"')
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _respond(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


# ---------------------------------------------------------------------------
# launch_ui
# ---------------------------------------------------------------------------

def launch_ui(step_path: str, port: int = 8765):
    print(f"Loading {step_path} ...")
    shape = load_step(step_path)
    unit = detect_step_unit(step_path)
    print(f"  Unit: {unit}")

    print("Analyzing faces ...")
    faces = analyze_faces(shape)
    print(f"  {len(faces)} faces")

    print("Triangulating ...")
    meshes = triangulate_faces(shape)

    print("Detecting bottom center ...")
    proposals = find_bottom_center_rules(faces)

    print("Detecting joint features ...")
    features = find_joint_features(faces)

    bb = Bnd_Box()
    BRepBndLib.Add_s(shape, bb)
    xmin, ymin, zmin, xmax, ymax, zmax = bb.Get()

    _Handler._shape = shape
    _Handler._step_path = step_path
    _Handler._unit = unit
    _Handler._model_data = {
        "filename": os.path.basename(step_path),
        "unit": unit,
        "faces": [asdict(f) for f in faces],
        "meshes": meshes,
        "proposals": [asdict(p) for p in proposals],
        "features": [asdict(f) for f in features],
        "bbox": {"min": [xmin, ymin, zmin], "max": [xmax, ymax, zmax]},
    }

    print_results(step_path, faces, proposals, features, unit)

    url = f"http://localhost:{port}"
    print(f"Viewer at {url}")
    print("Press Ctrl+C to stop.\n")
    webbrowser.open(url)
    server = HTTPServer(("0.0.0.0", port), _Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
        server.server_close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Find bottom center of STEP models (auto + interactive UI)")
    ap.add_argument("step_file", help="Path to .step file")
    ap.add_argument("--ui", action="store_true",
                    help="Launch interactive 3D viewer")
    ap.add_argument("-o", "--output", metavar="FILE",
                    help="Save analysis JSON")
    ap.add_argument("-e", "--export-step", metavar="FILE",
                    help="Auto-export transformed STEP (best proposal)")
    ap.add_argument("--x-rotate", type=int, default=0, metavar="DEG",
                    help="Rotate around Z by DEG degrees (e.g. 90, 180, 270)")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    if not os.path.isfile(args.step_file):
        print(f"Error: not found: {args.step_file}", file=sys.stderr)
        sys.exit(1)

    if args.ui:
        launch_ui(args.step_file, args.port)
        return

    print(f"Loading {args.step_file} ...")
    shape = load_step(args.step_file)
    unit = detect_step_unit(args.step_file)
    print(f"  Unit: {unit}")
    faces = analyze_faces(shape)
    proposals = find_bottom_center_rules(faces)
    features = find_joint_features(faces)
    print_results(args.step_file, faces, proposals, features, unit)

    if args.output:
        result = {
            "file": os.path.basename(args.step_file), "unit": unit,
            "proposals": [asdict(p) for p in proposals],
            "features": [asdict(f) for f in features],
            "best_proposal": asdict(proposals[0]) if proposals else None,
        }
        with open(args.output, "w") as f:
            json.dump(result, f, indent=2)
        print(f"Analysis saved to {args.output}")

    if args.export_step:
        if not proposals:
            print("Error: no proposal found; use --ui", file=sys.stderr)
            sys.exit(1)
        best = proposals[0]
        print(f"Applying transform ({best.label}, x_rotate={args.x_rotate}) ...")
        ok, record = export_transformed_step(
            args.step_file, best.center, best.z_axis, args.export_step,
            x_rot_deg=args.x_rotate, unit=unit)
        if ok:
            rec_path = args.export_step.replace(".step", "_transform.json")
            with open(rec_path, "w") as f:
                json.dump(record, f, indent=2)
            print(f"Exported: {args.export_step}")
            print(f"Record:   {rec_path}")
        else:
            print("Error: STEP export failed", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()
