#!/usr/bin/env python3
"""Turn the season's field CAD (a STEP AP242 assembly, as FIRST publishes it) into the simulator's
low-poly field model, field.json, next to replay.html under TeamCode/src/test/resources.

    python3 tools/field/step_to_field.py "BIOBUZZ_Full Field.20260912.step"

No dependencies beyond Python 3. The STEP file is walked directly: every product's placement is
composed down the assembly tree, and every solid is reduced to the points that bound it (its
B-rep vertices, its circular and elliptical edges sampled along their arcs, and the control points
of its spline edges, which bound the splines). Bolts, brackets and the other hardware are dropped
by name, and what is left is simplified to the general shape of each part: the convex hull of its
support points in a few dozen directions, so a box stays a box and a pipe becomes an octagonal
prism. Game pieces become balls, and the gaffer tape becomes marks on the floor.

The model is in the simulator's field frame, Road Runner's: inches, origin at the centre of the
field, +x away from the audience, +y to the audience's left, +z up. The CAD is in metres with +y
up and the audience at +z, which is where the hives' audience-facing cells hang.

What the robot collides with is the footprint of each field element below the robot's height:
the frame's legs and feet, and the flowers. The hives hang above the robot and are only drawn:
each cell as its flat panels, seen through and outlined in its alliance's colour. The pollen on
the floor in the open is marked loose, for the simulator to roll.
"""
import json
import math
import os
import re
import sys

INCH = 0.0254
ROBOT_HEIGHT_IN = 18  # SimRobot.ROBOT_SIZE_IN: what stands lower than this is in the robot's way
PANEL_AREA = 40  # square inches: a flat face this big on a hive cell is one of its panels
OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'TeamCode', 'src', 'test', 'resources',
                   'org', 'firstinspires', 'ftc', 'teamcode', 'sim', 'field.json')

# Hardware and the parts the simulator models on its own (the floor, the walls); none of it is drawn.
SKIP = re.compile(r'screw|fhts|nut\b|washer|rivet|rivnut|bolt|spacer|bearing|cable tie|plug|\bpin\b|'
                  r'sticker|damper|pivot damper|flower.*bracket|hinge|panel link|strap|clip|under tile|peanut|'
                  r'fastener|quick release|soft tiles|perimeter|rail|side glass', re.I)
# Alliance colours for the parts the CAD leaves uncoloured (the goal ribs, the nectar).
BLUE, RED, DARK = '#1651b0', '#c62828', '#2f2f2f'


# --- reading STEP ---------------------------------------------------------------------------------

def parse_args(s):
    """A STEP argument list like "('',#12,(1.,2.,3.),.T.)" as nested Python values; #12 is ('ref', 12)."""
    pos = 0
    n = len(s)

    def value():
        nonlocal pos
        while s[pos] == ' ':
            pos += 1
        c = s[pos]
        if c == '(':
            pos += 1
            items = []
            while True:
                while s[pos] in ' \n':
                    pos += 1
                if s[pos] == ')':
                    pos += 1
                    return items
                items.append(value())
                while s[pos] in ' \n':
                    pos += 1
                if s[pos] == ',':
                    pos += 1
        if c == "'":
            end = pos + 1
            while True:
                end = s.index("'", end)
                if end + 1 < n and s[end + 1] == "'":
                    end += 2
                    continue
                break
            text = s[pos + 1:end].replace("''", "'")
            pos = end + 1
            return text
        if c == '#':
            m = re.match(r'#(\d+)', s[pos:])
            pos += m.end()
            return ('ref', int(m.group(1)))
        m = re.match(r'\.[A-Z_0-9]+\.', s[pos:])
        if m:
            pos += m.end()
            return m.group(0)
        if c == '$' or c == '*':
            pos += 1
            return None
        m = re.match(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', s[pos:])
        if m:
            pos += m.end()
            return float(m.group(0))
        raise ValueError('unexpected %r at %d in %r' % (c, pos, s[:80]))

    return value()


class Step:
    """The entities of a STEP file by id, parsed on demand."""

    def __init__(self, path):
        self.e = {}
        self.cache = {}
        with open(path, encoding='latin-1') as f:
            data = f.read()
        start = data.index('\nDATA;') + 6
        end = data.index('\nENDSEC;', start)
        # A record starts at a line beginning with '#'; continuation lines do not.
        for rec in re.split(r'\n(?=#\d+=)', data[start:end]):
            rec = rec.strip()
            if not rec.startswith('#'):
                continue
            eq = rec.index('=')
            eid = int(rec[1:eq])
            rest = rec[eq + 1:].rstrip(';').replace('\n', '')
            if rest.startswith('('):
                # a complex entity: (TYPE1(args)TYPE2(args)...)
                parts = re.findall(r'([A-Z_0-9]+)\((.*?)\)(?=[A-Z_]|\)$)', rest[1:-1] + ')')
                self.e[eid] = ('COMPLEX', {name: '(' + args + ')' for name, args in parts})
            else:
                paren = rest.index('(')
                self.e[eid] = (rest[:paren], rest[paren:])

    def type(self, eid):
        return self.e[eid][0]

    def args(self, eid):
        if eid not in self.cache:
            t, a = self.e[eid]
            self.cache[eid] = {k: parse_args(v) for k, v in a.items()} if t == 'COMPLEX' else parse_args(a)
        return self.cache[eid]

    def by_type(self, *types):
        return [i for i, (t, _) in self.e.items() if t in types]

    def point(self, eid):
        return tuple(self.args(eid)[1])

    def placement(self, eid):
        """AXIS2_PLACEMENT_3D as (x axis, y axis, z axis, origin)."""
        a = self.args(eid)
        o = self.point(a[1][1])
        z = norm(tuple(self.args(a[2][1])[1])) if a[2] else (0, 0, 1)
        x = tuple(self.args(a[3][1])[1]) if a[3] else ((1, 0, 0) if abs(z[0]) < 0.9 else (0, 1, 0))
        x = norm(sub(x, scale(z, dot(x, z))))
        return (x, cross(z, x), z, o)


# --- vectors and placements -----------------------------------------------------------------------

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(v):
    n = math.sqrt(dot(v, v))
    return (v[0] / n, v[1] / n, v[2] / n)


def apply(m, p):
    x, y, z, o = m
    return (o[0] + p[0] * x[0] + p[1] * y[0] + p[2] * z[0],
            o[1] + p[0] * x[1] + p[1] * y[1] + p[2] * z[1],
            o[2] + p[0] * x[2] + p[1] * y[2] + p[2] * z[2])


def apply_dir(m, d):
    return apply((m[0], m[1], m[2], (0, 0, 0)), d)


def compose(a, b):
    """a after b."""
    return (apply_dir(a, b[0]), apply_dir(a, b[1]), apply_dir(a, b[2]), apply(a, b[3]))


def invert(m):
    x, y, z, o = m
    rot = ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))
    return rot + (apply_dir(rot + ((0, 0, 0),), scale(o, -1)),)


IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0))


# --- the points that bound a solid ----------------------------------------------------------------

def arc_points(m, r1, r2, start, end, same_sense):
    """Points along a circle's or ellipse's arc from the start vertex to the end vertex, the way
    the edge runs; the whole turn when they coincide."""
    x, y, z, c = m

    def angle(p):
        d = sub(p, c)
        return math.atan2(dot(d, y) / r2, dot(d, x) / r1)

    a0, a1 = angle(start), angle(end)
    if math.dist(start, end) < 1e-9:
        sweep = 2 * math.pi
    else:
        sweep = (a1 - a0) % (2 * math.pi) if same_sense else -((a0 - a1) % (2 * math.pi))
    n = max(2, int(abs(sweep) / (2 * math.pi) * 16) + 1)
    return [apply(m, (r1 * math.cos(a0 + sweep * k / n), r2 * math.sin(a0 + sweep * k / n), 0))
            for k in range(n + 1)]


def solid_points(st, solid):
    pts = []
    seen = set()
    shell = st.args(solid)[1][1]
    for face_ref in st.args(shell)[1]:
        face = face_ref[1]
        if st.type(face) not in ('ADVANCED_FACE', 'FACE_SURFACE'):
            continue
        for bound_ref in st.args(face)[1]:
            loop = st.args(bound_ref[1])[1][1]
            if st.type(loop) == 'VERTEX_LOOP':
                pts.append(st.point(st.args(st.args(loop)[1][1])[1][1]))
                continue
            for oe_ref in st.args(loop)[1]:
                edge = st.args(oe_ref[1])[3][1]
                if edge in seen:
                    continue
                seen.add(edge)
                ea = st.args(edge)
                start, end = (st.point(st.args(v)[1][1]) for v in (ea[1][1], ea[2][1]))
                pts += [start, end]
                curve = ea[3][1]
                ct = st.type(curve)
                if ct in ('CIRCLE', 'ELLIPSE'):
                    ca = st.args(curve)
                    r1 = ca[2]
                    r2 = ca[3] if ct == 'ELLIPSE' else r1
                    pts += arc_points(st.placement(ca[1][1]), r1, r2, start, end, ea[4] == '.T.')
                elif ct == 'B_SPLINE_CURVE_WITH_KNOTS':
                    pts += [st.point(cp[1]) for cp in st.args(curve)[2]]
                elif ct == 'COMPLEX' and 'B_SPLINE_CURVE' in st.args(curve):
                    pts += [st.point(cp[1]) for cp in st.args(curve)['B_SPLINE_CURVE'][1]]
    return pts


def solid_faces(st, solid, least_area):
    """The solid's flat faces of at least least_area (in the file's units squared): each as its
    outward normal and its outer boundary's vertices in order."""
    out = []
    shell = st.args(solid)[1][1]
    for face_ref in st.args(shell)[1]:
        face = face_ref[1]
        fa = st.args(face)
        if st.type(face) not in ('ADVANCED_FACE', 'FACE_SURFACE') or st.type(fa[2][1]) != 'PLANE':
            continue
        m = st.placement(st.args(fa[2][1])[1][1])
        n = m[2] if fa[3] == '.T.' else scale(m[2], -1)
        ring = None
        for bound_ref in fa[1]:
            loop = st.args(bound_ref[1])[1][1]
            if st.type(loop) != 'EDGE_LOOP':
                continue
            candidate = []
            for oe_ref in st.args(loop)[1]:
                oe = st.args(oe_ref[1])
                ea = st.args(oe[3][1])
                candidate.append(st.point(st.args(ea[1][1] if oe[4] == '.T.' else ea[2][1])[1][1]))
            if ring is None or st.type(bound_ref[1]) == 'FACE_OUTER_BOUND':
                ring = candidate
        if not ring:
            continue
        twice = (0.0, 0.0, 0.0)
        for a, b in zip(ring, ring[1:] + ring[:1]):
            c = cross(a, b)
            twice = (twice[0] + c[0], twice[1] + c[1], twice[2] + c[2])
        if abs(dot(twice, n)) / 2 >= least_area:
            out.append((n, ring))
    return out


def solid_colours(st):
    """Solid id -> '#rrggbb' from the STYLED_ITEMs, for the solids the CAD colours."""
    colours = {}
    for si in st.by_type('STYLED_ITEM'):
        a = st.args(si)
        for psa in a[1]:
            for ss in st.args(psa[1])[0]:
                if st.type(ss[1]) != 'SURFACE_STYLE_USAGE':
                    continue
                for sfa in st.args(st.args(ss[1])[1][1])[1]:
                    for fasc in st.args(st.args(sfa[1])[0][1])[1]:
                        col = st.args(fasc[1])[1]
                        if isinstance(col, tuple) and st.type(col[1]) == 'COLOUR_RGB':
                            colours[a[2][1]] = '#%02x%02x%02x' % tuple(int(round(v * 255)) for v in st.args(col[1])[1:4])
    return colours


# --- the assembly ---------------------------------------------------------------------------------

def placed_solids(st):
    """Every solid in the assembly, placed: (path of product names from the root, solid name,
    colour or None, world points in inches, CAD frame, and its flat faces of {@link PANEL_AREA}
    or more as (normal, ring of points) in the same frame)."""
    product_name = {}
    for pd in st.by_type('PRODUCT_DEFINITION'):
        product_name[pd] = st.args(st.args(st.args(pd)[2][1])[2][1])[1]
    pds_owner = {pds: st.args(pds)[2][1] for pds in st.by_type('PRODUCT_DEFINITION_SHAPE')}
    pd_rep = {}
    for sdr in st.by_type('SHAPE_DEFINITION_REPRESENTATION'):
        a = st.args(sdr)
        pd_rep[pds_owner[a[0][1]]] = a[1][1]
    rep_solids = {}
    for srr in st.by_type('SHAPE_REPRESENTATION_RELATIONSHIP'):
        a = st.args(srr)
        rep, brep = a[2][1], a[3][1]
        if st.type(brep) == 'ADVANCED_BREP_SHAPE_REPRESENTATION':
            rep_solids.setdefault(rep, []).extend(
                s[1] for s in st.args(brep)[1] if st.type(s[1]) == 'MANIFOLD_SOLID_BREP')
    # An occurrence's transformation: the child's placement in the parent, relative to the parent's origin.
    transform = {}
    for cdsr in st.by_type('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
        rel, pds = st.args(cdsr)[0][1], st.args(cdsr)[1][1]
        idt = st.args(rel)['REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION'][0][1]
        p1, p2 = st.args(idt)[2][1], st.args(idt)[3][1]
        transform[pds_owner[pds]] = compose(st.placement(p2), invert(st.placement(p1)))
    children = {}
    for nauo in st.by_type('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
        a = st.args(nauo)
        children.setdefault(a[3][1], []).append((nauo, a[4][1], a[1] or a[0]))
    placed = {c for lst in children.values() for _, c, _ in lst}
    roots = [pd for pd in product_name if pd not in placed]
    colours = solid_colours(st)
    local_points = {}
    local_faces = {}
    out = []

    def walk(pd, m, path):
        for solid in rep_solids.get(pd_rep.get(pd), []):
            if solid not in local_points:
                local_points[solid] = solid_points(st, solid)
                local_faces[solid] = solid_faces(st, solid, PANEL_AREA * INCH * INCH)
            out.append((path, st.args(solid)[0], colours.get(solid),
                        [scale(apply(m, p), 1 / INCH) for p in local_points[solid]],
                        [(apply_dir(m, n), [scale(apply(m, p), 1 / INCH) for p in ring]) for n, ring in local_faces[solid]]))
        for nauo, child, occurrence in children.get(pd, []):
            name = product_name[child]
            if occurrence and occurrence != name:
                name += ' [' + occurrence + ']'
            walk(child, compose(m, transform.get(nauo, IDENTITY)), path + [name])

    for r in roots:
        walk(r, IDENTITY, [product_name[r]])
    return out


# --- low-poly shapes ------------------------------------------------------------------------------

def support_directions():
    """The 26 directions of the axes and their diagonals, plus every 22.5 degrees round the
    horizon so that upright round things keep a round footprint."""
    dirs = set()
    for x in (-1, 0, 1):
        for y in (-1, 0, 1):
            for z in (-1, 0, 1):
                if (x, y, z) != (0, 0, 0):
                    dirs.add(norm((x, y, z)))
    for k in range(16):
        a = k * math.pi / 8
        dirs.add((round(math.cos(a), 12), round(math.sin(a), 12), 0.0))
    return sorted(dirs)


DIRECTIONS = support_directions()


def support_points(points):
    """The point farthest along each direction: the vertices of a hull that boxes the points the
    way a few dozen planes can."""
    out = set()
    for d in DIRECTIONS:
        best = max(points, key=lambda p: dot(p, d))
        out.add(tuple(round(c, 3) for c in best))
    return sorted(out)


def hull3d(points, eps=1e-6):
    """The convex hull of a few points as outward-facing triangles of indices, incrementally."""
    pts = list(points)
    p0 = min(range(len(pts)), key=lambda i: pts[i])
    p1 = max(range(len(pts)), key=lambda i: math.dist(pts[i], pts[p0]))
    line = sub(pts[p1], pts[p0])

    def off_line(i):
        c = cross(line, sub(pts[i], pts[p0]))
        return dot(c, c)

    p2 = max(range(len(pts)), key=off_line)
    if off_line(p2) < eps:
        return None
    n = cross(line, sub(pts[p2], pts[p0]))

    def off_plane(i):
        return abs(dot(n, sub(pts[i], pts[p0])))

    p3 = max(range(len(pts)), key=off_plane)
    if off_plane(p3) < eps * math.sqrt(dot(n, n)):
        return None
    faces = [(p0, p1, p2), (p0, p2, p3), (p0, p3, p1), (p1, p3, p2)]
    centre = scale(tuple(map(sum, zip(pts[p0], pts[p1], pts[p2], pts[p3]))), 0.25)

    def normal(f):
        return cross(sub(pts[f[1]], pts[f[0]]), sub(pts[f[2]], pts[f[0]]))

    faces = [f if dot(normal(f), sub(centre, pts[f[0]])) < 0 else (f[0], f[2], f[1]) for f in faces]
    for i in range(len(pts)):
        if i in (p0, p1, p2, p3):
            continue
        visible = [f for f in faces if dot(norm(normal(f)), sub(pts[i], pts[f[0]])) > eps]
        if not visible:
            continue
        edges = set()
        for f in visible:
            for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
                edges.add((a, b))
        horizon = [(a, b) for a, b in edges if (b, a) not in edges]
        faces = [f for f in faces if f not in visible] + [(a, b, i) for a, b in horizon]
    return faces


def hull2d(points):
    """Andrew's monotone chain: the convex hull of 2D points, counter-clockwise."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 1e-9:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 1e-9:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def outside_by(p, verts, tris):
    """How far p stands outside the hull: its largest distance beyond any face's plane, or zero."""
    worst = 0.0
    for t in tris:
        n = norm(cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]])))
        worst = max(worst, dot(n, sub(p, verts[t[0]])))
    return worst


def decimate(verts, tol):
    """Drop hull vertices one at a time, the least consequential first, while the shape without
    them stays within tol of the vertex dropped: a rounded corner becomes a corner."""
    verts = list(verts)
    hulls = {}

    def without(i):
        if i not in hulls:
            rest = verts[:i] + verts[i + 1:]
            tris = hull3d(rest)
            hulls[i] = None if tris is None else outside_by(verts[i], rest, tris)
        return hulls[i]

    while len(verts) > 4:
        errors = [(without(i), i) for i in range(len(verts))]
        error, i = min((e, i) for e, i in errors if e is not None)
        if error > tol:
            break
        verts.pop(i)
        hulls = {}
    return verts


def polygons(points):
    """A solid's low-poly shape: its hull's coplanar triangles merged into convex polygons, each a
    list of vertex indices counter-clockwise seen from outside. Returns (vertices, faces). The
    bigger the part, the coarser its shape, within a fraction of its thinnest dimension so a plate
    stays a plate: a quarter inch on a pipe, an inch on a hive cell."""
    verts = support_points(points)
    tris = hull3d(verts)
    if tris is None:
        return None
    # The part's thickness: how far the hull reaches from its flattest side.
    thickness = min(max(-dot(norm(cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))), sub(v, verts[t[0]]))
                        for v in verts) for t in tris)
    extent = max(max(p[i] for p in verts) - min(p[i] for p in verts) for i in range(3))
    if thickness < 0.75:
        return plate(verts, tris, thickness)
    verts = decimate(verts, min(0.25 + 0.03 * extent, 0.3 * thickness))
    tris = hull3d(verts)
    if tris is None:
        return None
    planes = {}
    for t in tris:
        n = norm(cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]])))
        key = (tuple(round(c, 3) for c in n), round(dot(n, verts[t[0]]), 2))
        planes.setdefault(key, (n, set()))[1].update(t)
    faces = []
    for (n, idx) in planes.values():
        u = norm(cross(n, (0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0)))
        v = cross(n, u)
        flat = {(round(dot(verts[i], u), 6), round(dot(verts[i], v), 6)): i for i in idx}
        ring = hull2d(list(flat))
        if len(ring) >= 3:
            faces.append([flat[p] for p in ring])
    used = sorted({i for f in faces for i in f})
    renumber = {old: new for new, old in enumerate(used)}
    return [verts[i] for i in used], [[renumber[i] for i in f] for f in faces]


def plate(verts, tris, thickness):
    """A thin part as its outline extruded: the hull of its points projected onto its flattest
    side, simplified, at both faces. Returns (vertices, faces) like {@link polygons}."""
    flattest = min(tris, key=lambda t: max(-dot(norm(cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))), sub(v, verts[t[0]])) for v in verts))
    n = norm(cross(sub(verts[flattest[1]], verts[flattest[0]]), sub(verts[flattest[2]], verts[flattest[0]])))
    u = norm(cross(n, (0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0)))
    v = cross(n, u)
    ring = simplify(hull2d([(round(dot(p, u), 4), round(dot(p, v), 4)) for p in verts]), 0.25)
    top = max(dot(p, n) for p in verts)
    bottom = min(dot(p, n) for p in verts)
    lift = lambda a, b, d: (u[0] * a + v[0] * b + n[0] * d, u[1] * a + v[1] * b + n[1] * d, u[2] * a + v[2] * b + n[2] * d)
    k = len(ring)
    out = [lift(a, b, top) for a, b in ring] + [lift(a, b, bottom) for a, b in ring]
    faces = [list(range(k)), list(range(2 * k - 1, k - 1, -1))]
    for i in range(k):
        j = (i + 1) % k
        faces.append([i + k, j + k, j, i])
    return out, faces


def footprint_below(vertices, faces, height):
    """The convex footprint of the shape's part below the given height: its vertices down there
    and where its edges cross that height. None when nothing of it is that low."""
    low = [(p[0], p[1]) for p in vertices if p[2] <= height]
    for f in faces:
        for a, b in zip(f, f[1:] + f[:1]):
            pa, pb = vertices[a], vertices[b]
            if (pa[2] - height) * (pb[2] - height) < 0:
                t = (height - pa[2]) / (pb[2] - pa[2])
                low.append((pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1])))
    if not low:
        return None
    return simplify(hull2d([(round(x, 3), round(y, 3)) for x, y in low]))


def simplify(ring, tol=0.05):
    """Drop a polygon's vertices that lie within tol of the edge between their neighbours."""
    out = list(ring)
    changed = True
    while changed and len(out) > 3:
        changed = False
        for i in range(len(out)):
            a, b, c = out[i - 1], out[i], out[(i + 1) % len(out)]
            ab = (c[0] - a[0], c[1] - a[1])
            length = math.hypot(*ab)
            if length < 1e-9 or abs((b[0] - a[0]) * ab[1] - (b[1] - a[1]) * ab[0]) / length < tol:
                out.pop(i)
                changed = True
                break
    return out


# --- the field ------------------------------------------------------------------------------------

def to_field(p):
    """CAD (x, y up, z toward the audience), inches -> Road Runner (x forward, y left, z up)."""
    return (-p[2], -p[0], p[1])


def clean(name):
    """'am-5853-Blue Hive <1>' -> 'Blue Hive <1>'; 'am-5869: Hive Goal Top Skin' -> 'Hive Goal Top Skin'."""
    name = re.sub(r'^am-\d+[a-z]?(?:-[A-Za-z0-9]+(?=:))?:?[-\s]*', '', name)
    name = re.sub(r'__Fillet\d+$', '', name)
    return re.sub(r'\s+', ' ', name).strip()


def colour_for(name, cad):
    if re.search(r'april tag', name, re.I):
        return '#4a4a4a'
    if cad and cad not in ('#e6e6e6',) or not re.search(r'blue|red', name, re.I):
        return cad or DARK
    return BLUE if re.search(r'blue', name, re.I) else RED


def dedupe_panels(panels):
    """A sheet's two sides are one panel: of the faces on the same plane, keep the biggest,
    as one flat polygon (the face's outline as a convex ring, simplified)."""
    by_plane = {}
    for panel in panels:
        n = panel['normal']
        offset = dot(n, panel['ring'][0])
        # Either side's normal names the plane: take the one pointing up, or forward.
        sign = 1 if (n[2], n[0], n[1]) > (0, 0, 0) else -1
        key = (panel['group'], panel['cell'], tuple(round(sign * c, 2) for c in n), round(sign * offset / 0.5) * 0.5)
        by_plane.setdefault(key, []).append(panel)
    out = []
    for key, group in sorted(by_plane.items()):
        panel = max(group, key=lambda p: len(p['ring']))
        n = panel['normal']
        u = norm(cross(n, (0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0)))
        v = cross(n, u)
        d = dot(n, panel['ring'][0])
        flat = simplify(hull2d([(round(dot(p, u), 4), round(dot(p, v), 4)) for p in panel['ring']]), 0.25)
        ring = [[round(u[i] * a + v[i] * b + n[i] * d, 2) for i in range(3)] for a, b in flat]
        out.append({'group': panel['group'], 'name': panel['cell'] + ' / ' + panel['name'], 'colour': panel['colour'],
                    'surface': True, 'vertices': ring, 'faces': [list(range(len(ring)))]})
    return out


def is_loose(piece, size, obstacles):
    """A game piece that rests on the floor inside the walls and in the open is the robot's to push."""
    x, y, z = piece['centre']
    r = piece['radius']
    if z > r + 0.25 or abs(x) > size / 2 - r / 2 or abs(y) > size / 2 - r / 2:
        return False
    for obstacle in obstacles:
        ring = obstacle['footprint']
        if all((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) > -r for a, b in zip(ring, ring[1:] + ring[:1])):
            return False
    return True


def build(step_path):
    st = Step(step_path)
    solids = placed_solids(st)
    tiles = [p for path, _, _, pts, _ in solids if 'Soft Tiles' in path[1] for p in pts]
    size = round(2 * max(abs(p[0]) for p in tiles), 2)
    wall = round(max(p[1] for path, name, _, pts, _ in solids if 'Perimeter' in path[1] and 'Rail' in name for p in pts), 2)

    elements, panels, pieces, tape = [], [], [], []
    # A hive cell is its flat panels, seen through; every other kept part is its own solid shape.
    for path, name, cad_colour, pts, faces in solids:
        top = clean(path[1]) if len(path) > 1 else clean(path[0])
        if re.search(r'gaffer tape', name, re.I):
            ring = simplify(hull2d([(round(x, 2), round(y, 2)) for x, y, _ in map(to_field, pts)]))
            tape.append({'colour': colour_for(name, cad_colour), 'footprint': [list(p) for p in ring]})
            continue
        if re.search(r'pollen|nectar', name, re.I):
            f = [to_field(p) for p in pts]
            lo = [min(c) for c in zip(*f)]
            hi = [max(c) for c in zip(*f)]
            pieces.append({'name': clean(name), 'colour': colour_for(name, cad_colour),
                           'centre': [round((a + b) / 2, 2) for a, b in zip(lo, hi)],
                           'radius': round(max(b - a for a, b in zip(lo, hi)) / 2, 2)})
            continue
        if any(SKIP.search(n) for n in path[1:] + [name]) or 'Soft Tiles' in path[1]:
            continue
        if 'Hive' in top and len(path) > 2 and 'Cell' in path[2] and 'April Tag' not in name:
            for n, ring in faces:
                panels.append({'group': top, 'cell': clean(path[2]), 'name': clean(name), 'colour': colour_for(path[2], None),
                               'normal': to_field(n), 'ring': [to_field(p) for p in ring]})
            continue
        elements.append({'group': top, 'name': clean(name), 'colour': colour_for(name, cad_colour), 'cad': pts})

    out_elements, obstacles, seen = [], {}, {}
    for e in elements:
        shape = polygons([to_field(p) for p in e['cad']])
        if shape is None:
            continue
        verts, faces = shape
        out_elements.append({'group': e['group'], 'name': e['name'], 'colour': e['colour'],
                             'vertices': [[round(c, 2) for c in v] for v in verts], 'faces': faces})
        # The frame is driven through between its legs, so each of its parts blocks on its own.
        key = e['group']
        if key.startswith('Frame'):
            seen[e['name']] = seen.get(e['name'], 0) + 1
            key += ' / ' + e['name'] + ' <%d>' % seen[e['name']]
        low = footprint_below(verts, faces, ROBOT_HEIGHT_IN)
        if low and all(abs(x) < size / 2 and abs(y) < size / 2 for x, y in low):
            obstacles.setdefault(key, []).extend(low)
    out_obstacles = [{'name': k, 'footprint': [list(p) for p in simplify(hull2d(v))]} for k, v in obstacles.items()]
    for panel in dedupe_panels(panels):
        out_elements.append(panel)
    for piece in pieces:
        piece['loose'] = is_loose(piece, size, out_obstacles)
    return {
        'source': os.path.basename(step_path),
        'size': size,
        'wallHeight': wall,
        'elements': sorted(out_elements, key=lambda e: (e['group'], e['name'])),
        'obstacles': sorted(out_obstacles, key=lambda o: o['name']),
        'pieces': sorted(pieces, key=lambda p: (p['name'], p['centre'])),
        'tape': sorted(tape, key=lambda t: (t['colour'], t['footprint'])),
    }


def main():
    if len(sys.argv) != 2:
        sys.exit('usage: step_to_field.py <field.step>')
    field = build(sys.argv[1])
    with open(OUT, 'w') as f:
        json.dump(field, f, separators=(',', ':'))
        f.write('\n')
    print('%s: %d elements, %d obstacles, %d pieces, %d tape marks; field %.2f in, walls %.2f in' % (
        os.path.relpath(OUT), len(field['elements']), len(field['obstacles']), len(field['pieces']),
        len(field['tape']), field['size'], field['wallHeight']))


if __name__ == '__main__':
    main()
