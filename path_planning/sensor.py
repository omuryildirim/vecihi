# -*- coding: utf-8 -*=

import math
import random

import director.vtkAll as vtk
import numpy as np
from director.debugVis import DebugData


class RaySensor(object):

    """Ray sensor."""

    def __init__(self, num_rays=20, radius=10, min_angle=-90, max_angle=90, z_distance=2.5, bottles=None):
        """Constructs a RaySensor.

        Args:
            num_rays: Number of rays.
            radius: Max distance of the rays.
            min_angle: Minimum angle of the rays in degrees.
            max_angle: Maximum angle of the rays in degrees.
        """
        self._num_rays = num_rays
        self._radius = radius
        self._min_angle = math.radians(min_angle)
        self._max_angle = math.radians(max_angle)

        self.z_distance = z_distance

        self._locator = None
        self._bottles = bottles
        self._state = [0., 0., 0.]  # x, y, theta

        self._hit = np.zeros(self._num_rays)
        self._distances = np.zeros(self._num_rays)
        self._intersections = [[0, 0, 0] for i in range(self._num_rays)]

        self._update_rays(self._state[2])

    @property
    def distances(self):
        """Array of distances measured by each ray."""
        normalized_distances = [
            self._distances[i] / self._radius if self._hit[i] else 1.0
            for i in range(self._num_rays)
        ]
        return normalized_distances

    def has_collided(self, max_distance=0.05):
        """Returns whether a collision has occured or not.

        Args:
            max_distance: Threshold for collision distance.
        """
        for hit, distance in zip(self._hit, self._distances):
            if hit and distance <= max_distance:
                return True

        return False

    def set_locator(self, locator):
        """Sets the vtk cell locator.

        Args:
            locator: Cell locator.
        """
        self._locator = locator


    def update(self, x, y, theta):
        """Updates the sensor's readings.

        Args:
            x: X coordinate.
            y: Y coordinate.
            theta: Yaw.
        """
        self._update_rays(theta)
        origin = np.array([x, y, self.z_distance])
        self._state = [x, y, theta]

        if self._locator is None:
            return

        for i in range(self._num_rays):
            hit, dist, inter = self._cast_ray(origin, origin + self._rays[i])
            self._hit[i] = hit
            self._distances[i] = dist
            self._intersections[i] = inter

    def _update_rays(self, theta):
        """Updates the rays' readings.

        Args:
            theta: Yaw.
        """
        r = self._radius
        angle_step = (self._max_angle - self._min_angle) / (self._num_rays - 1)
        self._rays = [
            np.array([
                r * math.cos(theta + self._min_angle + i * angle_step),
                r * math.sin(theta + self._min_angle + i * angle_step),
                0
            ])
            for i in range(self._num_rays)
        ]

    def _cast_ray(self, start, end):
        """Casts a ray and determines intersections and distances.

        Args:
            start: Origin of the ray.
            end: End point of the ray.

        Returns:
            Tuple of (whether it intersected, distance, intersection).
        """
        tolerance = 0.0                 # intersection tolerance
        pt = [0.0, 0.0, 0.0]            # coordinate of intersection
        distance = vtk.mutable(0.0)     # distance of intersection
        pcoords = [0.0, 0.0, 0.0]       # location within intersected cell
        subID = vtk.mutable(0)          # subID of intersected cell

        hit = self._locator.IntersectWithLine(start, end, tolerance,
                                              distance, pt, pcoords, subID)

        return hit, distance, pt

    def to_polydata(self, bottle_detected=False):
        """Converts the sensor to polydata."""
        d = DebugData()
        origin = np.array([self._state[0], self._state[1], self.z_distance])
        for hit, intersection, ray in zip(self._hit,
                                          self._intersections,
                                          self._rays):
            if hit:
                color = [1., 0.45882353, 0.51372549]
                endpoint = intersection
            else:
                color = [0., 0.6, 0.58823529]
                endpoint = origin + ray
            d.addLine(origin, endpoint, color=color, radius=0.05)

        # add vision sensor
        center = [self._state[0], self._state[1], 0]
        axis = [0, 0, 1]
        if bottle_detected:
            color = [0., 0.8, 0.]
        else:
            color = [0., 0.6, 0.58823529]

        d.addCircle(center, axis, 5, color=color)

        return d.getPolyData()

    def check_bottles(self):
        range_x = range(int(self._state[0]) - int(np.ceil(self.z_distance)), int(self._state[0]) + int(np.ceil(self.z_distance)))
        range_y = range(int(self._state[1]) - int(np.ceil(self.z_distance)), int(self._state[1]) + int(np.ceil(self.z_distance)))
        for index, bottle in enumerate(self._bottles):
            if int(bottle[0]) in range_x and int(bottle[1]) in range_y:
                return self.precision(), index

        return False, -1

    @staticmethod
    def precision(precision=60):
        w = [precision, 100-precision]
        i = [True, False]
        r = random.uniform(0, 100)

        # loop through a list of inputs and max cutoff values, returning
        # the first value for which the random num r is less than the cutoff value
        for n, v in map(None, i, [sum(w[:x + 1]) for x in range(len(w))]):
            if r < v:
                return n
