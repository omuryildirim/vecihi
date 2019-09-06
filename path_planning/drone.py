# -*- coding: utf-8 -*-

from copy import deepcopy

import numpy as np
from director import ioUtils, filterUtils
from director import vtkAll as vtk
from director.debugVis import DebugData


class Drone(object):
    """Robot."""

    def __init__(self, velocity=10.0, scale=0.5, exploration=0.5,
                 model="drone.obj", move_data=None, path=None, altitude=0.,
                 battery=4500.0, total_current=8000.0, return_base=True, base=None):
        """Constructs a Robot.

        Args:
            :param velocity: Velocity of the robot in the forward direction.
            :param scale: Scale of the model.
            :param exploration: Exploration rate.
            :param model: Object model to use.
            :param move_data: Visit points related to each cell.
            :param path: Path of drone.
            :param altitude: Altitude of drone.
            :param battery: Battery power of drone mAh.
            :param total_current: Mean current usage of drone.
            :param return_base: If robot needs to return base at the end of mission.
            :param base: Base location coordinates.
        """
        if base is None:
            base = [0, 0]
        self._target = (0, 0)
        self._exploration = exploration

        """
        If drone is working on task or not.
        """
        self.working = True

        """
        Power related parameters
        """
        self._battery = battery
        self._battery_consumption = 0.
        self._battery_percentage = 0.
        self._current = total_current
        self._unit_power_consumption = self._current * 1 / 3600
        self._safety_limit_for_battery = 0.32 * self._battery
        self._return_base = return_base
        self._base = base
        self.emergency_land = False
        self.emergency_base_return = False

        """
        Render robot in simulator.
        """
        t = vtk.vtkTransform()
        t.Scale(scale, scale, scale)
        t.RotateX(90.0)
        polydata = ioUtils.readPolyData(model)
        polydata = filterUtils.transformPolyData(polydata, t)

        """
        Initialize data for motion
        """
        self._state = np.array([0., 0., 0.])
        self.altitude = altitude
        self._dt = 1.0 / 30.0
        self._velocity = float(velocity)
        self._unit_shift = self.velocity * self._dt
        self._unit_angle_shift = self._dt * np.pi / 5
        self._raw_polydata = polydata
        self._polydata = polydata
        self._sensors = []
        self._moveData = move_data
        self._path = path
        self._target_cell_index = 0
        self._target_cell = 6
        self._target_move_index = 0
        self._target_point = None
        self.cell_change = True
        self._last_target_point = None
        self._obstacle_ahead = False
        self.total_motion_distance = 0

        self._target_acquisition_movements = {
            "turn_face": False,
            "get_closer": False,
            "rotate": False,
            "total_angular_movement": 0.,
            "circle_completed": False,
            "rotation_angle": self._dt * np.pi / 15
        }

        self.directions = {
            "left": np.pi,
            "top": np.pi / 2,
            "right": 0,
            "bottom": -np.pi / 2
        }

        self.init_move_data()

    def init_move_data(self):
        if self._moveData is not None:
            self._target_cell = self._path[self._target_cell_index]
            self._target_point = self._moveData[self._target_cell][self._target_move_index]

    def init_emergency_restart(self):
        self._battery_consumption = 0.
        self._battery_percentage = 0.
        self.emergency_land = False
        self.emergency_base_return = False
        self.working = True


    @property
    def x(self):
        """X coordinate."""
        return self._state[0]

    @x.setter
    def x(self, value):
        """X coordinate."""
        next_state = self._state.copy()
        next_state[0] = float(value)
        self._update_state(next_state)

    @property
    def y(self):
        """Y coordinate."""
        return self._state[1]

    @y.setter
    def y(self, value):
        """Y coordinate."""
        next_state = self._state.copy()
        next_state[1] = float(value)
        self._update_state(next_state)

    @property
    def theta(self):
        """Yaw in radians."""
        return self._state[2]

    @theta.setter
    def theta(self, value):
        """Yaw in radians."""
        next_state = self._state.copy()
        next_state[2] = float(value) % (2 * np.pi)
        self._update_state(next_state)

    @property
    def velocity(self):
        """Velocity."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        """Velocity."""
        self._velocity = float(value)

    @property
    def sensors(self):
        """List of attached sensors."""
        return self._sensors

    def attach_sensor(self, sensor):
        """Attaches a sensor.

        Args:
            sensor: Sensor.
        """
        self._sensors.append(sensor)

    def _update_state(self, next_state, altitude_change=0.):
        """Updates the moving object's state.

        Args:
            next_state: New state.
        """
        t = vtk.vtkTransform()
        t.Translate([next_state[0], next_state[1], altitude_change])
        t.RotateZ(np.degrees(next_state[2]))
        self._polydata = filterUtils.transformPolyData(self._raw_polydata, t)
        self.total_motion_distance += np.sqrt(np.square(next_state[0] - self._state[0]) + np.square(next_state[1] - self._state[1]))
        self._state = next_state
        self.altitude += altitude_change
        list(map(lambda s: s.update(*self._state), self._sensors))

    @staticmethod
    def create_updator():
        """
        General state updator with empty values.
        """
        return [0., 0., 0.]

    def to_positioned_polydata(self):
        """Converts object to visualizable poly data.

        Note: Transformations have been already applied to this.
        """
        return self._polydata

    def to_polydata(self):
        """Converts object to visualizable poly data.

        Note: This is centered at (0, 0, 0) and is not rotated.
        """
        return self._raw_polydata

    def next_target(self):
        """
        Select next target from coverage move data.

        Select next target in cell if current target is not the last visit point in cell. If current target is last
        point in cell, then select next cell's first visit point as target.

        If current target is last visit point in last cell then return None. Because all visit points are visited.

        :return: If all coverage points visited.
        """
        self._last_target_point = deepcopy(self._target_point)

        if self._target_move_index < len(self._moveData[self._target_cell]) - 1:
            self._target_move_index += 1
        elif self._target_cell_index < len(self._path) - 1:
            self._target_cell_index += 1
            self._target_cell = self._path[self._target_cell_index]
            self._target_move_index = 0
            self.cell_change = True
        else:
            self._target_point = None
            self.working = False
            print("Coverage is done!")
            return

        self._target_point = self._moveData[self._target_cell][self._target_move_index]

    def re_add_last_target(self):
        """
        Select previous target from coverage move data.

        Function called once a bottle detected. Bottle will be next target until investigation is completed.
        After investigation phase next target function will be called. In order to not skip selected target, it should
        change target index to previous target.
        """
        if self._target_move_index != 0:
            self._target_move_index -= 1
        elif self._target_cell_index != 0:
            self._target_cell_index -= 1
            self._target_cell = self._path[self._target_cell_index]
            self._target_move_index = len(self._moveData[self._target_cell]) - 1
        else:
            self._target_move_index = -1

    def move_towards_target(self):
        """
        Calculate power consumption and move safely towards the target.

        Before moving target turn towards target. If drone faces through target then move in linear line through target.
        """
        self.calculate_power_consumption()

        if self.safety_check():
            angle = self.calculate_angle_between_target()
            if self.turn(angle):
                if self.move_linear():
                    self.next_target()
            return True
        return False

    def move_towards_bottle(self, bottle):
        """
        Moves robot to certain distance from detected bottle location. Once position is stable makes circular
        movement around bottle.

        Args:
            :param bottle: Location of bottle.
        """
        self.calculate_power_consumption()

        if self.safety_check():
            state = deepcopy(self._state)

            dqdt = np.zeros_like(state)
            altitude_change = 0.

            degree_between_bottle = np.arctan((state[1] - bottle[1]) / (state[0] - bottle[0]))
            distance_to_bottle = np.sqrt(np.square(state[1] - bottle[1]) + np.square(state[0] - bottle[0]))

            if not self._target_acquisition_movements["turn_face"]:
                if state[0] > bottle[0]:
                    degree_between_bottle += np.pi
                if self.turn(degree_between_bottle):
                    self._target_acquisition_movements["turn_face"] = True
            elif not self._target_acquisition_movements["get_closer"]:
                if distance_to_bottle > 1.1:
                    dqdt[0] = np.cos(state[2]) * self._dt
                    dqdt[1] = np.sin(state[2]) * self._dt
                elif distance_to_bottle < 0.9:
                    dqdt[0] = -np.cos(state[2]) * self._dt
                    dqdt[1] = -np.sin(state[2]) * self._dt
                elif self.altitude - bottle[2] > 1.0:
                    altitude_change = -1 * self._dt
                else:
                    self._target_acquisition_movements["get_closer"] = True
            else:
                dqdt[2] = self._target_acquisition_movements["rotation_angle"]
                self._target_acquisition_movements["total_angular_movement"] += np.abs(dqdt[2])

                if self._target_acquisition_movements["total_angular_movement"] >= 2 * np.pi:
                    self._target_acquisition_movements["circle_completed"] = True
                    return False
                elif self.check_side_sensors_collided():
                    dqdt[2] = -1 * dqdt[2]
                    self._target_acquisition_movements["rotation_angle"] = dqdt[2]

                if state[0] < bottle[0]:
                    degree_between_bottle += np.pi

                dqdt[0] = bottle[0] + (distance_to_bottle * np.cos(degree_between_bottle + dqdt[2])) - self.x
                dqdt[1] = bottle[1] + (distance_to_bottle * np.sin(degree_between_bottle + dqdt[2])) - self.y

            return_state = self._state + dqdt
            self._update_state(return_state, altitude_change)

    def get_back_to(self, target):
        """
        Return to first position before bottle investigation.

        Args:
            :param target: Location at bottle detection.
        """
        self.calculate_power_consumption()

        if self.safety_check():
            dqdt = [0., 0., 0.]
            dqdt[0] = target[0] * self._dt
            dqdt[1] = target[1] * self._dt
            altitude_change = target[2] * self._dt
            return_state = self._state + dqdt
            self._update_state(return_state, altitude_change)

    def calculate_angle_between_target(self, target=None):
        """
        Calculate angle between move point or a certain target. Certain angle to turn towards selected target.

        Args:
            :param target: Target location if null select current move point as target.
        """
        if target is None:
            target = self._target_point

        if self.x != target[0]:
            degree_between_bottle = np.arctan((self.y - target[1]) / (self.x - target[0]))

            if self.x > target[0]:
                degree_between_bottle += np.pi

            return degree_between_bottle
        else:
            return self.theta

    def change_cell(self):
        """
        Transition between cells.
        Turn to start point of cell. Move towards start point. If an obstacle appears move towards Y till get rid of
        obstacle.
        """
        self.calculate_power_consumption()

        if self.safety_check():
            angle = self.calculate_angle_between_target()
            if self.turn(angle):
                obstacle_ahead = False
                direction = "y"
                for sensor in self.sensors:
                    if sensor.has_collided(max_distance=0.1):
                        collided = {"left": 0, "right": 0}
                        for index in range(0, len(sensor.distances)):
                            if index < 10 and sensor.distances[index] < 0.1:
                                collided["left"] += 1
                            elif index > 9 and sensor.distances[index] < 0.1:
                                collided["right"] += 1

                        angle_is_0_90 = (angle >= 0 and angle < 1.5707) or (angle <= 0 and angle > -4.7123)
                        angle_is_180_270 = (angle >= 3.1415 and angle < 4.7124) or (angle >= -3.1415 and angle < -1.5707)
                        angle_is_90_or_270 = (angle > 1.5533  and angle < 1.5882) or (angle > -4.7298 and angle < -4.6949) or (angle < 4.7298 and angle > 4.6949) or (angle < -1.5533  and angle > -1.5882)

                        if angle_is_90_or_270:
                            direction = "x"
                        elif angle_is_0_90 and angle_is_180_270:
                            if collided["left"] - collided["right"] > 0:
                                direction = "x"
                        elif collided["right"] - collided["left"] > 0:
                                direction = "x"

                        if abs(self.y - self._target_point[1]) + abs(self.x - self._target_point[0]) > 5:
                            obstacle_ahead = True
                if obstacle_ahead and self._target_cell != 6:
                    if direction == "y":
                        self.move_towards_y(self._target_point[1])
                    else:
                        self.move_towards_x(self._target_point[0])
                elif self.move_linear():
                    self.cell_change = False

    def move_linear(self, target=None):
        """
        Move in linear line with unit speed. Calculate distance between target point and drone. If unit movement bigger
        then distance then move as big as distance.

        Args:
            :param target: Target location, if null select current move point as target.
        """
        if target is None:
            target = self._target_point

        distance_to_target = np.sqrt(np.square(self.x - target[0]) + np.square(self.y - target[1]))
        if distance_to_target == 0:
            return True
        else:
            state_updator = self.create_updator()
            if distance_to_target < self._unit_shift:
                state_updator[0] = target[0] - self.x
                state_updator[1] = target[1] - self.y
                self._update_state(self._state + state_updator)
                return True
            else:
                state_updator[0] = self._unit_shift * np.cos(self.theta)
                state_updator[1] = self._unit_shift * np.sin(self.theta)
                self._update_state(self._state + state_updator)

        return False

    def turn(self, direction):
        """
        Set drones angle related to origin point of plane.

        Args:
            :param direction: Angle to turn.
        """
        state_updator = self.create_updator()
        if self._wrap_angles(direction - self.theta) > self._unit_angle_shift:
            state_updator[2] = self._unit_angle_shift
            self._update_state(self._state + state_updator)
            return False
        elif self._wrap_angles(direction - self.theta) < -self._unit_angle_shift:
            state_updator[2] = -self._unit_angle_shift
            self._update_state(self._state + state_updator)
            return False

        state_updator[2] = self._wrap_angles(direction - self.theta)
        self._update_state(self._state + state_updator)
        return True

    def move_to_x(self, target_x):
        """
        First turn towards target point and be parallel to x lines. Then move towards x
        horizon.

        Args:
            :param target_x: Target point x coordinate.
        """
        if self.x != target_x:

            on_right = target_x > self.x
            if on_right:
                if not self.turn(self.directions["right"]):
                    return False
            else:
                if not self.turn(self.directions["left"]):
                    return False

            state_updator = self.create_updator()
            if np.abs(target_x - self.x) < self._unit_shift:
                state_updator[0] = target_x - self.x
            elif on_right:
                state_updator[0] = self._unit_shift
            else:
                state_updator[0] = -self._unit_shift

            self._update_state(self._state + state_updator)
            return False
        return True

    def move_to_y(self, target_y):
        """
        First turn towards target point and be parallel to y lines. Then move towards y
        horizon.

        Args:
            :param target_y: Target point y coordinate.
        """
        if self.y != target_y:

            on_top = target_y > self.y
            if on_top:
                if not self.turn(self.directions["top"]):
                    return False
            else:
                if not self.turn(self.directions["bottom"]):
                    return False

            state_updator = self.create_updator()
            if np.abs(target_y - self.y) < self._unit_shift:
                state_updator[1] = target_y - self.y
            elif on_top:
                state_updator[1] = self._unit_shift
            else:
                state_updator[1] = -self._unit_shift

            self._update_state(self._state + state_updator)
            return False
        return True

    def move_towards_x(self, target_x):
        """
        Only move towards x coordinate. Move on x plane without relation to angle.

        Args:
            :param target_x: Target point x coordinate.
        """
        if self.x != target_x:

            on_right = target_x > self.x
            state_updator = self.create_updator()
            if np.abs(target_x - self.x) < self._unit_shift:
                state_updator[0] = target_x - self.x
            elif on_right:
                state_updator[0] = self._unit_shift
            else:
                state_updator[0] = -self._unit_shift

            self._update_state(self._state + state_updator)
            return False
        return True

    def move_towards_y(self, target_y):
        """
        Only move towards y coordinate. Move on y plane without relation to angle.

        Args:
            :param target_y: Target point y coordinate.
        """
        if self.y != target_y:

            on_top = target_y > self.y
            state_updator = self.create_updator()
            if np.abs(target_y - self.y) < self._unit_shift:
                state_updator[1] = target_y - self.y
            elif on_top:
                state_updator[1] = self._unit_shift
            else:
                state_updator[1] = -self._unit_shift

            self._update_state(self._state + state_updator)
            return False
        return True

    def land(self):
        """
        Safety land. Only called if power is under 32%. Try to set altitude to 0.
        """
        if self.altitude != 0:
            if self.altitude < self._unit_shift:
                self._update_state(self._state, -self.altitude)
            else:
                self._update_state(self._state, -self._unit_shift)
            return False
        return True

    def _return_to_base(self):
        """
        Safety return. Only called if battery under certain level. Return to base station.
        """
        if self._target_point != self._last_target_point:
            self._last_target_point = deepcopy(self._target_point)
            print("ALERT - Safely returning to base because of low power!")

        angle = self.calculate_angle_between_target(target=self._base)
        if self.turn(angle):
            if self.move_linear(target=self._base):
                self._target_point = None
                self.working = False
                print("Returned to base because of low power! Recharge to complete coverage.")
                print("Last target was x:{0} y:{1}".format(self._last_target_point[0], self._last_target_point[1]))
                self.emergency_base_return = True

    def focus_bottle(self, bottle):
        """
        Investigation phase of detected bottle. Try to label bottle on each movement step. Labelling precision change
        according to distance to bottle.

        Args:
            :param bottle: Bottle location.
            :return: label assumption and investigation status
        """
        self.move_towards_bottle(bottle)
        distance_to_bottle = np.sqrt(
            np.square(self.y - bottle[1]) + np.square(self.x - bottle[0]) + np.square(self.altitude - bottle[2]))
        precision = 0.6 * np.e / np.exp(distance_to_bottle)
        assumption = self.sensors[0].precision(precision)
        investigation_completed = deepcopy(self._target_acquisition_movements["circle_completed"])

        if assumption or investigation_completed:
            self._target_acquisition_movements = {
                "turn_face": False,
                "get_closer": False,
                "rotate": False,
                "total_angular_movement": 0.,
                "circle_completed": False,
                "rotation_angle": self._dt * np.pi / 15
            }
        return assumption, investigation_completed

    def set_target(self, target):
        """
        Setting private target point from outside function.
        """
        self._target = target

    def at_target(self, threshold=0):
        """Return whether the robot has reached its target.

        Args:
            :param threshold: Target distance threshold.
            :return: True if target is reached.
        """
        return (abs(self._state[0] - self._target[0]) <= threshold and
                abs(self._state[1] - self._target[1]) <= threshold)

    @staticmethod
    def _wrap_angles(angle):
        """
        Normalize angle to 2Pi range.

        :param angle: angle value to normalize.
        :return:
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def safety_check(self):
        """
        Main function to check drones safety rules.

        :return: True if drone is safe, False if drone has an emergency.
        """
        if self._battery - self._battery_consumption <= self._safety_limit_for_battery:
            print("Power alert, battery is at critical level. Safety landing started.")
            if self.land():
                print("Landed. Please pick me up!")
                print("Longitude: {0} Altitude:{1}".format(self.x, self.y))
                self.emergency_land = True
                self.working = False
            return False
        elif self.return_to_base_if_power_is_low():
            self.emergency_land = True
            return False
        return True

    def calculate_power_consumption(self):
        """
        Calculate power consumption on unit time. Power consumption calculated from current. It's not reflective to
        action drone takes. Can be improved in this way.

        Print if battery percentage drop 5%.
        """
        self._battery_consumption += self._dt * self._unit_power_consumption
        self._battery_percentage = 100 - 100 * self._battery_consumption / self._battery
        if self._battery_percentage % 5 < 0.002:
            print("Battery: %{}".format(np.floor(self._battery_percentage)))

    def return_to_base_if_power_is_low(self):
        """
        If base station is defined and if total of power need to return base and battery safety power is equals to
        current power then break every action and return to base station.

        :return: True if needs to return base, else is False
        """
        if self._return_base:
            distance_to_base = np.sqrt(np.square(self._base[0] - self.x) + np.square(self._base[1] - self.y))
            total_power_need = (distance_to_base / self.velocity) * self._unit_power_consumption

            if self._battery - total_power_need - self._battery_consumption < self._safety_limit_for_battery + 2:
                self._return_to_base()
                return True
        return False

    def check_side_sensors_collided(self):
        """
        Check lidar sensors on drone's side. One is on left and one is on right with 90 degrees angle.

        :return: True if side sensors collided, else is False
        """
        for index in [-1, 0]:
            for sensor in self.sensors:
                if sensor._hit[index] and sensor.distances[index] <= 0.15:
                    return True
        return False


class Obstacle(object):
    """Obstacle."""

    def __init__(self, points, polyline=True, circle=None):
        """Constructs an Obstacle.

        Args:
        :type points: point array for polyline obstacle
        :type polyline: if obstacle a polyline or not
        :type circle: if obstacle in circular shape or not
        """
        data = DebugData()
        if polyline:
            polydata = self.add_vtk_polygon(points)
            polydata = filterUtils.appendPolyData([polydata])
        elif circle is not None:
            center = [0, 0, 0]
            axis = [0, 0, 1]  # Upright cylinder.
            data.addCylinder(center, axis, 0.3, 0.1)
            polydata = data.getPolyData()
        else:
            center = [points[0], points[1], -1]
            axis = [0, 0, 1]  # Upright cylinder.
            data.addCircle(center, axis, 5)
            polydata = data.getPolyData()

        self._state = np.array([0., 0., 0.])
        self.altitude = 0.
        self._velocity = 0.
        self._raw_polydata = polydata
        self._polydata = polydata

    @property
    def x(self):
        """X coordinate."""
        return self._state[0]

    @x.setter
    def x(self, value):
        """X coordinate."""
        next_state = self._state.copy()
        next_state[0] = float(value)
        self._update_state(next_state)

    @property
    def y(self):
        """Y coordinate."""
        return self._state[1]

    @y.setter
    def y(self, value):
        """Y coordinate."""
        next_state = self._state.copy()
        next_state[1] = float(value)
        self._update_state(next_state)

    @property
    def theta(self):
        """Yaw in radians."""
        return self._state[2]

    @theta.setter
    def theta(self, value):
        """
        Yaw in radians
        """
        next_state = self._state.copy()
        next_state[2] = float(value) % (2 * np.pi)
        self._update_state(next_state)

    @property
    def velocity(self):
        """
        Velocity
        """
        return self._velocity

    def _update_state(self, next_state, altitude_change=0.):
        """
        Updates the moving object's state.

        Args:
            next_state: New state.
        """
        t = vtk.vtkTransform()
        t.Translate([next_state[0], next_state[1], altitude_change])
        t.RotateZ(np.degrees(next_state[2]))
        self._polydata = filterUtils.transformPolyData(self._raw_polydata, t)
        self._state = next_state
        self.altitude += altitude_change

    def to_positioned_polydata(self):
        """
        Converts object to visualization data.

        Note: Transformations have been already applied to this.
        """
        return self._polydata

    def to_polydata(self):
        """
        Converts object to visualization data.

        Note: This is centered at (0, 0, 0) and is not rotated.
        """
        return self._raw_polydata

    @staticmethod
    def add_vtk_polygon(vertices):
        """
        Create a 3d polygon data with vtk.
        :type vertices: vertices data of polygon
        """

        # Setup four points
        points = vtk.vtkPoints()
        for point in vertices:
            points.InsertNextPoint(point[0], point[1], point[2])

        # Create the polygon
        polygon = vtk.vtkPolygon()
        polygon.GetPointIds().SetNumberOfIds(4)  # make a quad
        polygon.GetPointIds().SetId(0, 0)
        polygon.GetPointIds().SetId(1, 1)
        polygon.GetPointIds().SetId(2, 2)
        polygon.GetPointIds().SetId(3, 3)

        # Add the polygon to a list of polygons
        polygons = vtk.vtkCellArray()
        polygons.InsertNextCell(polygon)

        # Create a PolyData
        polygon_poly_data = vtk.vtkPolyData()
        polygon_poly_data.SetPoints(points)
        polygon_poly_data.SetPolys(polygons)
        return polygon_poly_data
