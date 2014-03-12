"""
File Reader Module
"""


__author__ = 'Stefan Hechenberger <stefan@nortd.com>'

import sys

from .svg_reader import SVGReader
from .dxf_reader import DXFReader
from .ngc_reader import NGCReader
from .path_optimizers import optimize_all


def read_svg(svg_string, target_size, tolerance, forced_dpi=None, optimize=True):
    svgReader = SVGReader(tolerance, target_size)
    parse_results = svgReader.parse(svg_string, forced_dpi)
    if optimize:
        optimize_all(parse_results['boundarys'], tolerance)
    # {'boundarys':b, 'dpi':d, 'lasertags':l}
    return parse_results


def read_dxf(dxf_string, tolerance, optimize=True):
    dxfReader = DXFReader(tolerance)
    parse_results = dxfReader.parse(dxf_string)
    if optimize:
        optimize_all(parse_results['boundarys'], tolerance)

    min_x, max_x = sys.float_info.max, -sys.float_info.max
    min_y, max_y = sys.float_info.max, -sys.float_info.max

    for color, paths in parse_results['boundarys'].items():
        for path in paths:
            for vertex in path:
                min_x = min(min_x, vertex[0])
                max_x = max(max_x, vertex[0])

                min_y = min(min_y, vertex[1])
                max_y = max(max_y, vertex[1])

    # flip y-axis
    delta_x = max_x - min_x
    delta_y = max_y - min_y
    for color, paths in parse_results['boundarys'].items():
        for path in paths:
            for vertex in path:
                vertex[0] = vertex[0] - min_x
                vertex[1] = delta_y - (vertex[1] - min_y)

    return parse_results


def read_ngc(ngc_string, tolerance, optimize=True):
    ngcReader = NGCReader(tolerance)
    parse_results = ngcReader.parse(ngc_string)
    if optimize:
        optimize_all(parse_results['boundarys'], tolerance)
    return parse_results

