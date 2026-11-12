"""Exercise the installed converter with real PCD encodings and Nav2 image semantics."""
import math
from pathlib import Path
import struct
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest
from PIL import Image
import yaml


def write_pcd(path, points, encoding='ascii'):
    header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA {encoding}\n')
    if encoding == 'ascii':
        payload = ('\n'.join(' '.join(map(str, point)) for point in points)+'\n').encode()
    elif encoding == 'binary':
        payload = b''.join(struct.pack('<fff', *point) for point in points)
    else:
        # PCD compressed layout is field-major; valid LZF literal blocks need no extra library.
        raw = b''.join(struct.pack('<f', point[axis]) for axis in range(3) for point in points)
        compressed = b''.join(bytes([len(raw[i:i+32])-1])+raw[i:i+32]
                              for i in range(0, len(raw), 32))
        payload = struct.pack('<II', len(compressed), len(raw))+compressed
    path.write_bytes(header.encode()+payload)


def convert(tmp_path, points, *options, encoding='ascii'):
    pcd = tmp_path/'source.pcd'
    write_pcd(pcd, points, encoding)
    original = pcd.read_bytes()
    executable = Path(get_package_prefix('lunar_navigation'))/'lib/lunar_navigation/pcd_to_grid'
    result = subprocess.run([str(executable), '--input', str(pcd), '--output', str(tmp_path/'map'),
                             '--resolution', '0.05', '--margin', '0', *options],
                            capture_output=True, text=True)
    assert pcd.read_bytes() == original
    return result


def cell(image, metadata, x, y):
    column = math.floor((x-metadata['origin'][0])/metadata['resolution'])
    row = math.floor((y-metadata['origin'][1])/metadata['resolution'])
    return image.getpixel((column, image.height-1-row))


@pytest.mark.parametrize('encoding', ['ascii', 'binary', 'binary_compressed'])
def test_map_origin_image_orientation_and_obstacle_precedence(tmp_path, encoding):
    points = [(.025, .025, -.25), (.125, .025, -.05), (.125, .025, -.25),
              (.325, .325, -.25), (-.125, -.075, -.25), (.225, .025, 2.0)]
    result = convert(tmp_path, points, '--ground-z', '-0.25', encoding=encoding)
    assert result.returncode == 0, result.stderr
    metadata = yaml.safe_load((tmp_path/'map.yaml').read_text())
    assert metadata['image'] == 'map.pgm'
    assert metadata['origin'] == pytest.approx([-.15, -.1, 0])
    assert metadata['mode'] == 'trinary' and metadata['negate'] == 0
    with Image.open(tmp_path/'map.pgm') as image:
        assert cell(image, metadata, .025, .025) == 254
        assert cell(image, metadata, .125, .025) == 0
        assert cell(image, metadata, .325, .325) == 254
        assert cell(image, metadata, -.125, -.075) == 254
        assert cell(image, metadata, .225, .225) == 205
        assert cell(image, metadata, .225, .025) == 205  # Ceiling is not an obstacle.


def test_auto_ground_fits_start_floor_without_flattening_map(tmp_path):
    floor = [(.025+i*.1, .025+j*.1, -.23846+.02*(.025+i*.1))
             for i in range(12) for j in range(12)]
    # More ceiling points than floor points: the starting-IMU height rejects the ceiling.
    ceiling = [(x, y, 1.8) for x, y, _ in floor]*3
    wall = [(1.225, .025+i*.1, -.03846+.02*1.225) for i in range(12)]
    result = convert(tmp_path, floor+ceiling+wall)
    assert result.returncode == 0, result.stderr
    metadata = yaml.safe_load((tmp_path/'map.yaml').read_text())
    with Image.open(tmp_path/'map.pgm') as image:
        assert cell(image, metadata, .025, .025) == 254
        assert cell(image, metadata, 1.125, 1.125) == 254
        assert cell(image, metadata, 1.225, .025) == 0
    line = next(line for line in result.stdout.splitlines() if line.startswith('Ground z'))
    assert float(line.split(':')[1].split()[0]) == pytest.approx(-.23846, abs=1e-4)


def test_missing_ground_requests_explicit_height(tmp_path):
    result = convert(tmp_path, [(0., 0., 1.), (1., 0., 1.), (0., 1., 1.)])
    assert result.returncode != 0
    assert '--ground-z' in result.stderr
    assert not (tmp_path/'map.yaml').exists()


def test_invalid_resolution_is_rejected(tmp_path):
    result = convert(tmp_path, [(0., 0., -.25)], '--resolution', '0')
    assert result.returncode != 0
    assert not (tmp_path/'map.yaml').exists()
