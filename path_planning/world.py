# -*- coding: utf-8 -*=

import random

import numpy as np
from director.debugVis import DebugData

from drone import Obstacle


class World(object):
    """Base world."""

    def __init__(self, width, height, cells=None):
        """Construct an empty world.

        Args:
            width: Width of the field.
            height: Height of the field.
        """
        self._data = DebugData()

        self._width = width
        self._height = height
        self._add_boundaries()
        self._cells = cells

    def _add_boundaries(self):
        """
        Adds boundaries to the world
        """
        self._x_max, self._x_min = self._width / 2.0, -self._width / 2.0
        self._y_max, self._y_min = self._height / 2.0, -self._height / 2.0

        corners = list([
            (self._x_max, self._y_max, 0),  # Top-right corner.
            (self._x_max, self._y_min, 0),  # Bottom-right corner.
            (self._x_min, self._y_min, 0),  # Bottom-left corner.
            (self._x_min, self._y_max, 0),  # Top-left corner.
        ])

        # Loop back to beginning.
        corners.append(corners[0])

        for start, end in zip(corners, corners[1:]):
            self._data.addLine(start, end, radius=0.2)

    @staticmethod
    def get_bottle_name():
        """
        Give a name to bottle. With given probabilities.
        :return: Bttle name
        """
        beer_brands = list(["Paulaner", "Augustiner", "Franziskaner", "Krombacher", "Bitburger", "Beck's", "Warsteiner",
                           "Radeberger", "Erdinger"])

        brand_density = [25, 35, 7, 5, 3, 15, 3, 2, 10]
        r = random.uniform(0, 100)

        # loop through a list of inputs and max cutoff values, returning
        # the first value for which the random num r is less than the cutoff value
        for n, v in map(None, beer_brands, [sum(brand_density[:x + 1]) for x in range(len(brand_density))]):
            if r < v:
                return n

    def generate_bottles(self, count=10, seed=None):
        """
        Generates given number of bottles with random locations

        Args:
            :param count: Total bottle count.
            :param seed: Random seed, default: None.

        Yields:
            Obstacle, Bottle Name.
        """
        if seed is not None:
            np.random.seed(seed)

        for index in range(0, count):
            bounds = random.choice(self._cells)
            # bounds = self._cells[0]

            center_x_range = (self._x_min + bounds[0].x, self._x_min + bounds[1].x)
            # center_x_range = (self._x_min + bounds[0].x, self._x_min + bounds[0].x + 5)
            center_x = np.random.uniform(*center_x_range)

            center_y_range = (self._y_min + bounds[0].y, self._y_min + bounds[3].y)
            center_y = np.random.uniform(*center_y_range)
            theta = np.random.uniform(0., 360.)
            obstacle = Obstacle(None, polyline=False, circle=True)
            obstacle.x = center_x
            obstacle.y = center_y
            obstacle.theta = np.radians(theta)
            yield obstacle, self.get_bottle_name()

    def generate_bottle(self, position_x, position_y):
        """
        Generate one obstacele with given position

        Args:
            :param position_x: x coordinate of bottle.
            :param position_y: y coordinate of bottle.

        Returns:
            Obstacle.
        """

        theta = np.random.uniform(0., 360.)
        obstacle = Obstacle(None, polyline=False, circle=True)
        obstacle.x = position_x
        obstacle.y = position_y
        obstacle.theta = np.radians(theta)
        return obstacle

    def to_polydata(self):
        """
        Converts world to visualization data
        """
        return self._data.getPolyData()
