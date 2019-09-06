#!/usr/bin/env python

"""
This module contains code to plan coverage paths
"""

import math
from copy import deepcopy
from filecmp import cmp
from logging import error

from shapely.geometry import GeometryCollection
from shapely.geometry import MultiLineString, MultiPoint
from shapely.geometry import Point, LineString
from shapely.geos import TopologicalError


def generate_intersections(poly, width):
    """
    Get coverage lines inside a cell. Lines will be centered throught cell with padding of width/2.

    :param poly: Polygon data
    :param width: Width of distance between vertical lines
    :return: Intersected lines
    """
    initial_line = LineString([(poly.bounds[0] + width / 2.0, poly.bounds[1] - 1),
                               (poly.bounds[0] + width / 2.0, poly.bounds[3] + 1)])
    line = initial_line.parallel_offset(0, 'right')
    lines = []

    if poly.bounds[2] - poly.bounds[0] >= width / 2:
        iterations = int(math.ceil((poly.bounds[2] - poly.bounds[0] - width / 2.0) / width))
        for x in range(0, iterations):
            if poly.intersects(line):
                try:
                    bounded_line = deepcopy(line)
                    intersection_line = poly.intersection(line)

                    if intersection_line.bounds[1] != bounded_line.bounds[1]:
                        if intersection_line.bounds[1] > bounded_line.bounds[1]:
                            bounds = list(bounded_line.bounds)
                            bounds[1] = intersection_line.bounds[1] + width / 2.0
                        else:
                            bounds = list(bounded_line.bounds)
                            bounds[1] = intersection_line.bounds[1] - width / 2.0
                        bounded_line = LineString([tuple(bounds[:2]), tuple(bounds[2:])])

                    if intersection_line.bounds[3] != bounded_line.bounds[3]:
                        if intersection_line.bounds[3] > bounded_line.bounds[3]:
                            bounds = list(bounded_line.bounds)
                            bounds[3] = intersection_line.bounds[3] + width / 2.0
                        else:
                            bounds = list(bounded_line.bounds)
                            bounds[3] = intersection_line.bounds[3] - width / 2.0
                        bounded_line = LineString([tuple(bounds[:2]), tuple(bounds[2:])])

                except TopologicalError:
                    error("Problem looking for intersection.", exc_info=1)
                    continue
                lines.append(bounded_line)

            line = initial_line.parallel_offset(width * (x + 1), 'right')
    return lines


def sort_to(point, list_of_points):
    """
    Sorts a set of points by distance to a point

    :param point: Origin point
    :param list_of_points: Point list
    :return:
    """
    copy_list = deepcopy(list_of_points)
    copy_list.sort(key=lambda x: x.distance(Point(*point)))
    return copy_list


def get_furthest(line_point_list, origin):
    """
    Get a point along a line furthest away from a given point

    :param line_point_list: Point list of line
    :type origin: Origin
    """
    orig_point = Point(*origin)
    line_point_list = sorted(line_point_list, key=lambda x: orig_point.distance(Point(*x)))
    return line_point_list


def order_points(lines, initial_origin):
    """
    Return a list of points in a given coverage path order

    :param initial_origin: Origin point
    :param lines: List of lines in cell
    """
    origin = initial_origin
    results = []
    while True:
        if not len(lines):
            break
        lines = sort_to(origin, lines)
        f = lines.pop(0)
        if type(f) == GeometryCollection:
            continue
        if type(f) == MultiLineString:
            for ln in f:
                lines.append(ln)
            continue
        if type(f) == Point or type(f) == MultiPoint:
            continue
        xs, ys = f.xy
        ps = zip(xs, ys)
        (start, end) = get_furthest(ps, origin)
        results.append(start)
        results.append(end)
        origin = end
    return results


def decompose(polygon, origin=None, width=1.0):
    """
    Decompose the field into a list of points to cover the field.
    :param polygon: Polygon data
    :param width: Distance between two vertical line
    :param origin: Origin
    """
    p = generate_intersections(polygon, width)
    if origin is None:
        return order_points(p, polygon.bounds[0:2])
    else:
        return order_points(p, origin)
