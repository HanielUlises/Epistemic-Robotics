# Copyright 2026 Haniel Vásquez Morales
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The residence's furniture, read from the collision meshes the simulator uses.

The pieces are AWS RoboMaker's residential models (MIT-0), from the small
house world's package. What a robot can hit, and what its laser sees, is the
collision mesh, so that is what is read: each mesh is cut at the heights of a
TurtleBot3's body and laser, and the cut is rasterised into the occupancy grid
that tools/check_views.py casts rays over and the page draws.

    furniture.py               prints each model's footprint in its own frame

HOUSE_MODELS names the models directory when it is not the installed one.
"""

import functools
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

COLLADA = '{http://www.collada.org/2005/11/COLLADASchema}'
MODELS = os.environ.get('HOUSE_MODELS', os.path.expanduser(
    '~/plansys2_ws/install/aws_robomaker_small_house_world/share/aws_robomaker_small_house_world/models'))
PREFIX = 'aws_robomaker_residential_'
Z_LO, Z_HI = 0.03, 0.30            # a Burger's body and its laser, 0.17 m up


def _matrix(node):
    m = node.find(COLLADA + 'matrix')
    if m is not None:
        return np.fromstring(m.text, sep=' ').reshape(4, 4)
    t = np.eye(4)
    tr = node.find(COLLADA + 'translate')
    if tr is not None:
        t[:3, 3] = np.fromstring(tr.text, sep=' ')
    sc = node.find(COLLADA + 'scale')
    if sc is not None:
        t = t @ np.diag(list(np.fromstring(sc.text, sep=' ')) + [1.0])
    return t


@functools.lru_cache(maxsize=None)
def triangles(model):
    """The collision mesh of a model, as an (n, 3, 3) array in metres, model frame."""
    path = os.path.join(MODELS, PREFIX + model, 'meshes', f'aws_{model}_collision.DAE')
    root = ET.parse(path).getroot()
    unit = 1.0
    node = root.find(COLLADA + 'asset/' + COLLADA + 'unit')
    if node is not None:
        unit = float(node.get('meter'))
    up = root.find(COLLADA + 'asset/' + COLLADA + 'up_axis')
    up = up.text.strip() if up is not None else 'Z_UP'
    geometries = {}
    for geometry in root.iter(COLLADA + 'geometry'):
        mesh = geometry.find(COLLADA + 'mesh')
        if mesh is None:
            continue
        sources = {}
        for source in mesh.findall(COLLADA + 'source'):
            array = source.find(COLLADA + 'float_array')
            if array is not None:
                sources['#' + source.get('id')] = np.fromstring(array.text, sep=' ')
        vertices = mesh.find(COLLADA + 'vertices')
        position = None
        for inp in vertices.findall(COLLADA + 'input'):
            if inp.get('semantic') == 'POSITION':
                position = sources.get(inp.get('source'))
        points = position.reshape(-1, 3)
        faces = []
        for kind in ('triangles', 'polylist'):
            for tri in mesh.findall(COLLADA + kind):
                inputs = tri.findall(COLLADA + 'input')
                stride = max(int(i.get('offset')) for i in inputs) + 1
                offset = next(int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX')
                index = np.fromstring(tri.find(COLLADA + 'p').text, sep=' ', dtype=np.int64)
                verts = index.reshape(-1, stride)[:, offset]
                vcount = tri.find(COLLADA + 'vcount')
                if vcount is None:
                    faces.append(verts.reshape(-1, 3))
                    continue
                # A polylist's polygons may have more than three corners: fanned.
                k = 0
                fan = []
                for n in np.fromstring(vcount.text, sep=' ', dtype=np.int64):
                    poly = verts[k:k + n]
                    fan += [(poly[0], poly[i], poly[i + 1]) for i in range(1, n - 1)]
                    k += n
                if fan:
                    faces.append(np.array(fan, dtype=np.int64))
        if faces:
            geometries[geometry.get('id')] = (points, np.vstack(faces))

    out = []

    def walk(node, parent):
        t = parent @ _matrix(node)
        for inst in node.findall(COLLADA + 'instance_geometry'):
            key = inst.get('url').lstrip('#')
            if key in geometries:
                points, faces = geometries[key]
                placed = (np.c_[points, np.ones(len(points))] @ t.T)[:, :3]
                out.append(placed[faces])
        for child in node.findall(COLLADA + 'node'):
            walk(child, t)
    for scene in root.iter(COLLADA + 'visual_scene'):
        for node in scene.findall(COLLADA + 'node'):
            walk(node, np.eye(4))
    tri = np.vstack(out) * unit
    if up == 'Y_UP':
        tri = tri[..., [0, 2, 1]] * np.array([1, -1, 1])
    return tri


def placed(model, x, y, yaw, z=0.0):
    """The model's triangles at a pose in the world."""
    t = triangles(model)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return t @ rot.T + np.array([x, y, z])


def slice_segments(tri, z_lo=Z_LO, z_hi=Z_HI, step=0.05):
    """The triangles cut at heights between z_lo and z_hi, as 2D segments."""
    segs = []
    for z in np.arange(z_lo, z_hi + 1e-9, step):
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        za, zb, zc = a[:, 2] - z, b[:, 2] - z, c[:, 2] - z
        for (p, q, zp, zq), (r, s, zr, zs) in (((a, b, za, zb), (b, c, zb, zc)), ((b, c, zb, zc), (c, a, zc, za)),
                                               ((c, a, zc, za), (a, b, za, zb))):
            m = (zp * zq < 0) & (zr * zs < 0)
            if not m.any():
                continue
            t1 = (zp[m] / (zp[m] - zq[m]))[:, None]
            t2 = (zr[m] / (zr[m] - zs[m]))[:, None]
            u = p[m] + t1 * (q[m] - p[m])
            v = r[m] + t2 * (s[m] - r[m])
            segs.append(np.hstack([u[:, :2], v[:, :2]]))
    return np.vstack(segs) if segs else np.zeros((0, 4))


def footprint(model, x=0.0, y=0.0, yaw=0.0, z=0.0):
    """(x0, y0, x1, y1) of what a robot can hit, at a pose."""
    s = slice_segments(placed(model, x, y, yaw, z))
    if not len(s):
        return None
    xs, ys = np.r_[s[:, 0], s[:, 2]], np.r_[s[:, 1], s[:, 3]]
    return (xs.min(), ys.min(), xs.max(), ys.max())


if __name__ == '__main__':
    for name in sorted(d[len(PREFIX):] for d in os.listdir(MODELS) if d.startswith(PREFIX)):
        try:
            t = triangles(name)
            lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
            fp = footprint(name)
            fps = 'none at robot height' if fp is None else '[%.2f %.2f] x [%.2f %.2f]' % (fp[0], fp[2], fp[1], fp[3])
            print(f'{name:22s} x[{lo[0]:6.2f},{hi[0]:6.2f}] y[{lo[1]:6.2f},{hi[1]:6.2f}] z[{lo[2]:5.2f},{hi[2]:5.2f}]  '
                  f'cut {fps}')
        except Exception as e:  # noqa: BLE001
            print(f'{name:22s} ERR {e}')
