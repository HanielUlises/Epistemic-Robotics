"""Rasterise a world SDF into an occupancy grid.

The worlds are static geometry: axis-aligned boxes and upright cylinders. A
model whose top lies below ROBOT_CLEARANCE is floor marking rather than an
obstacle, which is what the goal disc in room 6 is.
"""
import math
import xml.etree.ElementTree as ET

ROBOT_CLEARANCE = 0.2


def _pose(el):
    p = (el.findtext('pose') or '0 0 0 0 0 0').split()
    vals = [float(v) for v in p] + [0.0] * (6 - len(p))
    return vals[0], vals[1], vals[2], vals[5]


def shapes(path):
    """(kind, cx, cy, yaw, params) for every collision worth avoiding."""
    world = ET.parse(path).getroot().find('world')
    out = []
    for model in world.findall('model'):
        mx, my, mz, myaw = _pose(model)
        for link in model.findall('link'):
            lx, ly, lz, lyaw = _pose(link)
            cx, cy, cz, yaw = mx + lx, my + ly, mz + lz, myaw + lyaw
            for col in link.findall('collision'):
                box = col.find('geometry/box/size')
                cyl = col.find('geometry/cylinder')
                if box is not None:
                    sx, sy, sz = [float(v) for v in box.text.split()]
                    if cz + sz / 2.0 < ROBOT_CLEARANCE:
                        continue
                    out.append(('box', cx, cy, yaw, (sx, sy)))
                elif cyl is not None:
                    r = float(cyl.findtext('radius'))
                    h = float(cyl.findtext('length'))
                    if cz + h / 2.0 < ROBOT_CLEARANCE:
                        continue
                    out.append(('cyl', cx, cy, yaw, (r,)))
    return out


class Grid:
    def __init__(self, min_x, min_y, max_x, max_y, resolution):
        self.res = resolution
        self.origin_x = min_x
        self.origin_y = min_y
        self.width = int(math.ceil((max_x - min_x) / resolution))
        self.height = int(math.ceil((max_y - min_y) / resolution))
        self.data = [0] * (self.width * self.height)

    def centre(self, col, row):
        return (self.origin_x + (col + 0.5) * self.res,
                self.origin_y + (row + 0.5) * self.res)

    def index(self, x, y):
        col = int((x - self.origin_x) / self.res)
        row = int((y - self.origin_y) / self.res)
        if 0 <= col < self.width and 0 <= row < self.height:
            return row * self.width + col
        return None

    def fill(self, shapes_, value=100, inflate=0.0):
        for row in range(self.height):
            for col in range(self.width):
                x, y = self.centre(col, row)
                for kind, cx, cy, yaw, p in shapes_:
                    dx, dy = x - cx, y - cy
                    if yaw:
                        c, s = math.cos(-yaw), math.sin(-yaw)
                        dx, dy = c * dx - s * dy, s * dx + c * dy
                    if kind == 'box':
                        if abs(dx) <= p[0] / 2 + inflate and abs(dy) <= p[1] / 2 + inflate:
                            self.data[row * self.width + col] = value
                            break
                    else:
                        if dx * dx + dy * dy <= (p[0] + inflate) ** 2:
                            self.data[row * self.width + col] = value
                            break

    def set_region(self, min_x, min_y, max_x, max_y, value):
        for row in range(self.height):
            for col in range(self.width):
                x, y = self.centre(col, row)
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    i = row * self.width + col
                    if self.data[i] != 100:
                        self.data[i] = value

    def cells_in(self, min_x, min_y, max_x, max_y, free_only=True):
        out = []
        for row in range(self.height):
            for col in range(self.width):
                x, y = self.centre(col, row)
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    i = row * self.width + col
                    if not free_only or self.data[i] == 0:
                        out.append(i)
        return out
