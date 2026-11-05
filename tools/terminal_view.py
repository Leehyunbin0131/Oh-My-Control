"""ANSI terminal drawing and a wire model derived from the reference URDF."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from wcwidth import wcwidth


BG = (12, 18, 27)
TEXT = (216, 229, 240)
MUTED = (108, 130, 151)
GREEN = (64, 221, 154)
CYAN = (81, 202, 245)
YELLOW = (255, 198, 99)
RED = (255, 103, 115)
BLUE = (81, 105, 131)


def rotation(rpy):
    r, p, y = np.radians(rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_matrix(element):
    matrix = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
        rpy = [math.degrees(float(v)) for v in element.get('rpy', '0 0 0').split()]
        matrix[:3, :3], matrix[:3, 3] = rotation(rpy), xyz
    return matrix


class RobotModel:
    def __init__(self, path=None):
        path = path or Path(__file__).resolve().parents[1] / 'resources/robot_base.urdf'
        root = ET.parse(path).getroot()
        links = {link.get('name'): link for link in root.findall('link')}
        poses = {'base_footprint': np.eye(4)}
        joints = list(root.findall('joint'))
        while joints:
            done = []
            for joint in joints:
                parent = joint.find('parent').get('link')
                if parent in poses:
                    poses[joint.find('child').get('link')] = poses[parent] @ origin_matrix(joint.find('origin'))
                    done.append(joint)
            if not done:
                raise ValueError('URDF link tree is incomplete')
            joints = [joint for joint in joints if joint not in done]
        self.lines = []
        for name, link in links.items():
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                edges, points = [], []
                if geometry.find('box') is not None:
                    sizes = np.array([float(v) for v in geometry.find('box').get('size').split()]) / 2
                    points = [sizes * np.array([x, y, z]) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
                    edges = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count('1') == 1]
                elif geometry.find('cylinder') is not None:
                    cyl = geometry.find('cylinder')
                    radius, half = float(cyl.get('radius')), float(cyl.get('length')) / 2
                    points = [[radius * math.cos(i * math.pi / 8), radius * math.sin(i * math.pi / 8), z]
                              for z in (-half, half) for i in range(16)]
                    edges = [(i + side * 16, (i + 1) % 16 + side * 16) for side in (0, 1) for i in range(16)]
                    edges += [(i, i + 16) for i in range(0, 16, 4)]
                if not points:
                    continue
                transform = poses[name] @ origin_matrix(collision.find('origin'))
                vertices = (transform @ np.c_[np.array(points), np.ones(len(points))].T).T[:, :3]
                color = CYAN if name == 'camera_link' else GREEN if name == 'lidar_link' else TEXT if name == 'base_link' else BLUE
                self.lines.extend((vertices[a], vertices[b], color) for a, b in edges)
        all_points = np.array([p for a, b, _ in self.lines for p in (a, b)])
        self.center = (all_points.min(axis=0) + all_points.max(axis=0)) / 2
        self.radius = max(np.linalg.norm(p - self.center) for p in all_points)


class Screen:
    def __init__(self, width, height, color='truecolor'):
        self.width, self.height, self.color = width, height, color
        self.cells = [[(' ', TEXT, BG) for _ in range(width)] for _ in range(height)]

    def put(self, x, y, char, fg=TEXT, bg=BG):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.cells[y][x] = (char, fg, bg)

    def text(self, x, y, value, fg=TEXT, limit=None):
        end = self.width if limit is None else min(self.width, x + limit)
        for char in str(value):
            size = wcwidth(char)
            if size <= 0:
                char, size = ' ', 1
            if x + size > end:
                break
            self.put(x, y, char, fg)
            if size == 2:
                self.put(x + 1, y, '', fg)
            x += size

    def panel(self, x, y, width, height, title):
        self.text(x, y, '+' + '-' * (width - 2) + '+', BLUE)
        self.text(x, y + height - 1, '+' + '-' * (width - 2) + '+', BLUE)
        for row in range(y + 1, y + height - 1):
            self.put(x, row, '|', BLUE)
            self.put(x + width - 1, row, '|', BLUE)
        self.text(x + 2, y, f' {title} ', CYAN, width - 4)

    @staticmethod
    def ansi_color(rgb, background=False, mode='truecolor'):
        if mode == '256':
            r, g, b = (round(v * 5 / 255) for v in rgb)
            return f'\x1b[{48 if background else 38};5;{16 + 36 * r + 6 * g + b}m'
        return f'\x1b[{48 if background else 38};2;{rgb[0]};{rgb[1]};{rgb[2]}m'

    def render(self, ansi=True):
        rows, current = [], None
        for row in self.cells:
            chunks = []
            for char, fg, bg in row:
                if not char:
                    continue
                style = (fg, bg)
                if ansi and self.color != 'none' and style != current:
                    chunks.append(self.ansi_color(fg, mode=self.color) + self.ansi_color(bg, True, self.color))
                    current = style
                chunks.append(char)
            rows.append(''.join(chunks))
        return '\n'.join(rows) + ('\x1b[0m' if ansi else '')


class DotPlot:
    def __init__(self, columns, rows):
        self.width, self.height = columns * 2, rows * 4
        self.dots = {}

    def dot(self, x, y, color=TEXT):
        x, y = round(x), round(y)
        if 0 <= x < self.width and 0 <= y < self.height:
            cell = (x // 2, y // 4)
            bit = ((0, 1, 2, 6), (3, 4, 5, 7))[x % 2][y % 4]
            old = self.dots.get(cell, (0, color))
            self.dots[cell] = (old[0] | (1 << bit), color)

    def line(self, a, b, color=TEXT):
        steps = int(max(abs(a[0] - b[0]), abs(a[1] - b[1]))) + 1
        for t in np.linspace(0, 1, min(steps, 2000)):
            self.dot(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), color)

    def draw(self, screen, x, y):
        for (col, row), (mask, color) in self.dots.items():
            screen.put(x + col, y + row, chr(0x2800 + mask), color)


def draw_robot(screen, box, model, attitude):
    x, y, width, height = box
    plot = DotPlot(width, height)
    angle, elevation = -0.9, 0.55
    right = np.array([-math.sin(angle), math.cos(angle), 0])
    up = np.array([-math.sin(elevation) * math.cos(angle), -math.sin(elevation) * math.sin(angle), math.cos(elevation)])
    scale = min(plot.width, plot.height) / (2.3 * model.radius)

    def project(point):
        point = attitude @ (point - model.center)
        return [plot.width / 2 + right @ point * scale, plot.height / 2 - up @ point * scale]

    for a, b, color in model.lines:
        plot.line(project(a), project(b), color)
    # The reference URDF uses +X as forward.
    origin = model.center + np.array([0, 0, 0.04])
    end = origin + np.array([0.24, 0, 0])
    plot.line(project(origin), project(end), YELLOW)
    for delta in ([0.18, 0.035, 0], [0.18, -0.035, 0]):
        plot.line(project(origin + delta), project(end), YELLOW)
    plot.draw(screen, x, y)


def draw_image(screen, box, pixels):
    x, y, width, height = box
    if pixels is None:
        screen.text(x + 1, y + height // 2, 'WAITING FOR LIVE IMAGE', MUTED, width - 2)
        return
    size = (width, height * 2)
    image = Image.fromarray(pixels)
    image.thumbnail(size)
    data = np.asarray(image)
    left, top = x + (width - image.width) // 2, y + (height - math.ceil(image.height / 2)) // 2
    for row in range(0, image.height, 2):
        for col in range(image.width):
            upper = tuple(int(v) for v in data[row, col])
            lower = tuple(int(v) for v in data[min(row + 1, image.height - 1), col])
            if screen.color == 'none':
                index = min(9, int(sum(upper) / 3 * 9 / 255))
                screen.put(left + col, top + row // 2, ' .:-=+*#%@'[index])
            else:
                screen.put(left + col, top + row // 2, '\u2580', upper, lower)


def draw_lidar(screen, box, sample, distance):
    x, y, width, height = box
    plot = DotPlot(width, height)
    cx, cy = plot.width / 2, plot.height / 2
    scale = (min(plot.width, plot.height) / 2 - 2) / distance
    for radius in (distance / 2, distance):
        points = [(cx + math.cos(a) * radius * scale, cy + math.sin(a) * radius * scale)
                  for a in np.linspace(0, 2 * math.pi, 65)]
        for a, b in zip(points, points[1:]):
            plot.line(a, b, BLUE)
    plot.line([cx, 1], [cx, plot.height - 2], BLUE)
    plot.line([1, cy], [plot.width - 2, cy], BLUE)
    for px, py in sample.get('points', []):
        if math.hypot(px, py) <= distance:
            plot.dot(cx - py * scale, cy - px * scale, YELLOW)
    plot.line([cx - 2, cy + 2], [cx, cy - 3], CYAN)
    plot.line([cx, cy - 3], [cx + 2, cy + 2], CYAN)
    plot.draw(screen, x, y)
    if not sample:
        screen.text(x + 1, y + height - 1, 'NO LIVE SCAN', RED, width - 2)
