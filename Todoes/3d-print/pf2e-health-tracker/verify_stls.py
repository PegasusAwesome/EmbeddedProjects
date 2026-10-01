"""Independent binary STL checks using only the Python standard library."""
from collections import Counter, defaultdict
from pathlib import Path
import json
import math
import struct

ROOT = Path(__file__).resolve().parent


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def check(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + 50*count, (path.name, 'Invalid binary size')
    edges = Counter()
    orientation = Counter()
    neighbors = defaultdict(set)
    seen_faces = set()
    volume = 0
    for index in range(count):
        raw = struct.unpack_from('<12fH', data, 84+50*index)
        normal = raw[:3]
        points = [tuple(raw[i:i+3]) for i in (3, 6, 9)]
        assert all(math.isfinite(c) for point in points for c in point)
        a, b, c = points
        area_vector = cross(sub(b, a), sub(c, a))
        assert dot(area_vector, area_vector) > 1e-18, (path.name, index, 'Degenerate face')
        assert dot(normal, area_vector) > 0, (path.name, index, 'Reversed normal')
        face = tuple(sorted(points))
        assert face not in seen_faces, (path.name, index, 'Duplicate triangle')
        seen_faces.add(face)
        volume += dot(a, cross(b, c))/6
        for start, end in ((a,b), (b,c), (c,a)):
            edge = tuple(sorted((start, end)))
            edges[edge] += 1
            orientation[edge] += 1 if start < end else -1
            neighbors[start].add(end)
            neighbors[end].add(start)
    assert all(n == 2 for n in edges.values()), (path.name, 'Open or nonmanifold edge')
    assert all(n == 0 for n in orientation.values()), (path.name, 'Inconsistent winding')
    remaining = set(neighbors)
    components = 0
    while remaining:
        components += 1
        stack = [remaining.pop()]
        while stack:
            for other in neighbors[stack.pop()]:
                if other in remaining:
                    remaining.remove(other)
                    stack.append(other)
    expected = 10 if path.name == 'hearts_10_flat.stl' else 1
    assert components == expected, (path.name, components, expected)
    mins = [min(v[i] for v in neighbors) for i in range(3)]
    maxs = [max(v[i] for v in neighbors) for i in range(3)]
    assert abs(mins[2]) < 1e-5, (path.name, 'Not resting on print bed')
    assert volume > 0
    return dict(triangles=count, connected_components=components,
                watertight=True, consistent_winding=True,
                duplicate_triangles=0, degenerate_triangles=0,
                volume_mm3=round(volume, 3),
                size_mm=[round(b-a, 4) for a, b in zip(mins, maxs)])


if __name__ == '__main__':
    results = {str(path.relative_to(ROOT)): check(path) for path in sorted(ROOT.rglob('*.stl'))}
    assert len(results) == 7, f'Expected seven STL files, found {len(results)}'
    (ROOT/'stl_checks.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))
