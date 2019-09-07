# -*- coding: utf-8 -*-

import argparse
import random
from copy import deepcopy

import codecs
import os
import numpy as np
from PythonQt import QtGui
from builtins import int
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director import vtkAll as vtk
from director.consoleapp import ConsoleApp
from director.debugVis import DebugData
from director.timercallback import TimerCallback

from drone import Drone, Obstacle
from sensor import RaySensor
from vertical_cell_decomposition import VerticalCellDecomposition
from world import World


class Simulator(object):
    """Simulator."""

    def __init__(self, simulator_world):
        """
        Constructs the simulator.

        Args:
            :type simulator_world: World
        """
        self._tick_count = 0
        self._timer = TimerCallback(targetFps=120)
        self.locator = vtk.vtkCellLocator()
        self._robots = []
        self._obstacles = []
        self._bottles = []
        self.bottle_locations = []
        self._world = simulator_world
        self._app = ConsoleApp()
        self._view = self._app.createView(useGrid=False)

        # performance tracker
        self._num_targets = 0

        self._coverage_frame = None
        self._bottle_found = False
        self._detected_bottle_index = None
        self._bottle_found_location = None
        self._external_target = False

        self._found_bottles = []

        self._total_run_count = 0
        self._text_file = None
        self.total_simulation_count = 15
        self._finished_simulation_count = 0

        self._initialize()

    def _initialize(self):
        """
        Initializes the simulation
        """
        # Add world to view.
        om.removeFromObjectModel(om.findObjectByName("world"))
        vis.showPolyData(self._world.to_polydata(), "world")

    @staticmethod
    def _add_poly_data(poly_data, frame_name, color):
        """
        Adds polydata to the simulation.

        Args:
            :param poly_data: Polydata.
            :param frame_name: Frame name.
            :param color: Color of object.
            :return: Frame
        """
        om.removeFromObjectModel(om.findObjectByName(frame_name))
        frame = vis.showPolyData(poly_data, frame_name, color=color)

        vis.addChildFrame(frame)
        return frame

    def add_target(self, target):
        """
        Adds poly data to the simulation.

        Args:
            :param target: Target data
        """
        data = DebugData()
        center = [target[0], target[1], -1]
        axis = [0, 0, 1]  # Upright cylinder.
        data.addCylinder(center, axis, 2, 3)
        om.removeFromObjectModel(om.findObjectByName("target"))
        self._add_poly_data(data.getPolyData(), "target", [0, 0.8, 0])

    def add_robot(self, robot_data):
        """
        Adds a robot to the simulation.

        Args:
            :param robot_data: Robot.
        """
        color = [0.4, 0.85098039, 0.9372549]
        frame_name = "robot{}".format(len(self._robots))
        frame = self._add_poly_data(robot_data.to_polydata(), frame_name, color)
        self._robots.append((robot_data, frame))
        self._update_moving_object(robot_data, frame)

    def add_obstacle(self, obstacle_data, bottle=False, brand="", color=None):
        """
        Adds an obstacle to the simulation.

        Args:
            :param obstacle_data: Obstacle
            :param bottle: If obstacle is bottle or not
            :param brand: Brand of bottle
            :param color: Color array of obstacle
        """
        if color is None:
            color = [1.0, 1.0, 1.0]

        if bottle:
            frame_name = "bottles{}".format(len(self._bottles))
            frame = self._add_poly_data(obstacle_data.to_polydata(), frame_name, color)
            self._bottles.append((obstacle_data, frame, brand))
            self.bottle_locations.append([obstacle_data.x, obstacle_data.y, obstacle_data.altitude])
        else:
            frame_name = "obstacle{}".format(len(self._obstacles))
            frame = self._add_poly_data(obstacle_data.to_polydata(), frame_name, color)
            self._obstacles.append((obstacle_data, frame))
        self._update_moving_object(obstacle_data, frame)

    def replace_bottle(self, bottle, frame_name, color):
        """
        Update bottle data on simulator.

        Args:
            :param bottle: If obstacle is bottle or not
            :param frame_name: Name of bottle frame
            :param color: Color array of obstacle
        """
        om.removeFromObjectModel(om.findObjectByName(frame_name))

        data = DebugData()
        center = [bottle.x, bottle.y, 0]
        axis = [0, 0, 1]  # Upright cylinder.
        data.addCylinder(center, axis, 0.3, 0.1)
        self._add_poly_data(data.getPolyData(), frame_name, color)

    def _update_moving_object(self, moving_object, frame, object_index=None):
        """
        Updates moving object's state.

        Args:
            :param moving_object: Moving object.
            :param frame: Corresponding frame.
        """
        t = vtk.vtkTransform()
        t.Translate(moving_object.x, moving_object.y, moving_object.altitude-0.7)
        t.RotateZ(np.degrees(moving_object.theta))
        frame.getChildFrame().copyFrame(t)

        if object_index is not None:
            for sensor in moving_object.sensors:
                sensor.z_distance = moving_object.altitude
                frame_name = "rays{}".format(object_index)
                self._update_sensor(sensor, frame_name)

    def _update_sensor(self, sensor, frame_name):
        """Updates sensor's rays.

        Args:
            sensor: Sensor.
            frame_name: Frame name.
        """
        vis.updatePolyData(sensor.to_polydata(bottle_detected=self._bottle_found), frame_name,
                           colorByName="RGB255")

    def update_locator(self):
        """
        Updates cell locator
        """
        d = DebugData()
        d.addPolyData(self._world.to_polydata())
        for obstacle_data, frame in self._obstacles:
            d.addPolyData(obstacle_data.to_positioned_polydata())
        self.locator.SetDataSet(d.getPolyData())
        self.locator.BuildLocator()

    def run(self, display, simulation_number):
        """
        Launches and displays the simulator.

        Args:
            display: Displays the simulator or not.
            :param simulation_number: Current simulation number.
        """
        self._finished_simulation_count = simulation_number
        self._text_file = open("./prints/" + args.input_file + "/" + str(self._finished_simulation_count) + "/" + "monitoring_summary" + str(self._total_run_count) + ".txt", "w")

        for bottle in self._bottles:
            self._text_file.write("Bottle is {0}, at location x: {1} y: {2}".format(bottle[2], bottle[0].x, bottle[0].y))
            self._text_file.write("\n")

        if display:
            widget = QtGui.QWidget()
            layout = QtGui.QVBoxLayout(widget)
            layout.addWidget(self._view)
            widget.showMaximized()
            # Set camera.
            applogic.resetCamera(viewDirection=[0.2, 0, -1])

        # Set timer.
        self._timer.callback = self.tick
        self._timer.start()

        self._app.start()

    def record(self):
        windowToImageFilter = vtk.vtkWindowToImageFilter()
        windowToImageFilter.SetInput(vis)
        windowToImageFilter.SetInputBufferTypeToRGBA()
        windowToImageFilter.ReadFrontBufferOff()
        windowToImageFilter.Update()

        writer = vtk.vtkAVIWriter()
        writer.SetInputConnection(windowToImageFilter.GetOutputPort())
        writer.SetFileName("test.avi")

    def tick(self):
        """
        Update simulation clock
        """
        self._tick_count += 1
        if self._robots[0][0].working:
            if self._bottle_found:
                for i, (robot_data, frame) in enumerate(self._robots):
                    assumption, investgation_completed = robot_data.focus_bottle(
                        self.bottle_locations[self._detected_bottle_index])
                    if assumption:
                        bottle = deepcopy(self._bottles[self._detected_bottle_index])
                        self.replace_bottle(bottle[0],
                                            ("bottles" + str(self._detected_bottle_index)),
                                            color=[0.0, 0.8, 0.0]
                                            )
                        self._bottle_found = False
                        self.bottle_locations.pop(self._detected_bottle_index)
                        self._bottles.pop(self._detected_bottle_index)
                        self._external_target = True
                        self._bottle_found_location = [
                            self._bottle_found_location[0] - deepcopy(robot_data.x),
                            self._bottle_found_location[1] - deepcopy(robot_data.y),
                            self._bottle_found_location[2] - deepcopy(robot_data.altitude),
                            30
                        ]
                        print("Target acquired, bottle label is collected")
                        self._found_bottles.append(bottle[2])
                        print("Beer is {0}. Total detected beer bottle count: {1}".format(bottle[2],
                                                                                          len(self._found_bottles)))

                        self._text_file.write("Beer is {0}. Total detected beer bottle count: {1}".format(bottle[2],
                                                                                          len(self._found_bottles)))
                        self._text_file.write("\n")
                    elif investgation_completed:
                        print("Target couldn't defined. Object is not a bottle.")
                        self._text_file.write("Target couldn't defined. Object is not a bottle.")
                        self._text_file.write("\n")

                        self._bottle_found = False
                        self.bottle_locations.pop(self._detected_bottle_index)
                        self._bottles.pop(self._detected_bottle_index)
                        self._external_target = True
                        self._bottle_found_location = [
                            self._bottle_found_location[0] - deepcopy(robot_data.x),
                            self._bottle_found_location[1] - deepcopy(robot_data.y),
                            self._bottle_found_location[2] - deepcopy(robot_data.altitude),
                            30
                        ]

                    self._update_moving_object(robot_data, frame, object_index=i)
            elif self._external_target:
                for i, (robot_data, frame) in enumerate(self._robots):
                    robot_data.get_back_to(self._bottle_found_location)
                    self._bottle_found_location[3] -= 1
                    self._update_moving_object(robot_data, frame, object_index=i)
                    if self._bottle_found_location[3] == 0:
                        self._external_target = False
                        robot_data.next_target()
            else:
                for i, (robot_data, frame) in enumerate(self._robots):
                    self._update_moving_object(robot_data, frame, object_index=i)
                    for sensor in robot_data.sensors:
                        sensor.set_locator(self.locator)

                    if args.show_covered_area:
                        curent_point = Obstacle([robot_data.x, robot_data.y], polyline=False)
                        if self._coverage_frame is not None:
                            vis.showPolyData(curent_point.to_polydata(), self._coverage_frame, color=[0, 0.8, 0])
                            print("coverage continues")
                        else:
                            self._coverage_frame = "coverage_frame"
                            self._add_poly_data(curent_point.to_polydata(), self._coverage_frame, [0, 0.8, 0])
                            print("coverage started")

                    if robot_data.cell_change:
                        robot_data.change_cell()
                    elif robot_data.move_towards_target():

                        for sensor in robot_data.sensors:
                            self._bottle_found, self._detected_bottle_index = sensor.check_bottles()

                        if self._bottle_found:
                            self._bottle_found_location = [deepcopy(robot_data.x), deepcopy(robot_data.y),
                                                           deepcopy(robot_data.altitude)]
                            robot_data.re_add_last_target()
                            print("Possible bottle detected. Further investigation started.")
                            print("Detection coordinate x: {0}, y: {1}".format(robot_data._state[0], robot_data._state[1]))
                            self._text_file.write("Detection coordinate x: {0}, y: {1}".format(robot_data._state[0], robot_data._state[1]))
                            self._text_file.write("\n")

                        if robot_data.at_target(threshold=0.1):
                            self._num_targets += 1
                            new_target = robot_data._target_point
                            robot_data.set_target(new_target)
                            if new_target is not None:
                                self.add_target(new_target)
        else:
            self._text_file.write("Total movement: {0} meter".format(self._robots[0][0].total_motion_distance))
            self._text_file.write("\n")
            self._text_file.write("Battery percentage: {0} %".format(self._robots[0][0]._battery_percentage))
            self._text_file.write("\n")
            self._text_file.write("Consumption: {0} mAh".format(self._robots[0][0]._battery_consumption))
            self._text_file.write("\n")
            self._text_file.write("Total time: {0} tick".format(self._tick_count))
            self._text_file.write("\n")

            if self._robots[0][0].emergency_land or self._robots[0][0].emergency_base_return:
                # store last location of robot
                new_text_file = open("./prints/" + args.input_file + "/" + str(self._finished_simulation_count) + "/" + "emergency_" + str(self._total_run_count) + ".txt", "w")
                new_text_file.write(str(self._robots[0][0]._target_cell_index))
                new_text_file.write("\n")
                new_text_file.write(str(self._robots[0][0]._target_move_index))
                new_text_file.close()

                self._text_file.write("\n")
                self._text_file.write("\n")
                self._text_file.write("emergency cell and point")
                self._text_file.write(str(self._robots[0][0]._target_cell_index))
                self._text_file.write("\n")
                self._text_file.write(str(self._robots[0][0]._target_move_index) + " - " +
                                str(len(self._robots[0][0]._moveData[self._robots[0][0]._target_cell])))

                # store bottle names and locations
                new_text_file = open("./prints/" + args.input_file + "/" + str(self._finished_simulation_count) + "/" + "bottle_locations.txt", "w")
                for bottle in self._bottles:
                    new_text_file.write("{0},{1},{2}".format(bottle[2], bottle[0].x, bottle[0].y))
                    new_text_file.write("\n")
                new_text_file.close()
                self._total_run_count += 1
                self._found_bottles = []
                self._tick_count = 0
                self.set_safe_position(self._robots[0][0])
                self._robots[0][0].init_emergency_restart()
                self._robots[0][0].altitude = 2.
                self._text_file.close()
                self._text_file = open("./prints/" + args.input_file + "/" + str(self._finished_simulation_count) + "/" + "monitoring_summary" + str(self._total_run_count) + ".txt", "w")
                return

            self._text_file.close()
            self._app.quit()

            # if self.total_simulation_count == self._finished_simulation_count:
            #     self._app.quit()
            # else:
            #     self.set_safe_position(self._robots[0][0])
            #     self._robots[0][0].init_emergency_restart()
            #     self._tick_count = 0
            #     self._obstacles = []
            #     self._bottles = []
            #     self.bottle_locations = []
            #
            #     # performance tracker
            #     self._num_targets = 0
            #
            #     self._coverage_frame = None
            #     self._bottle_found = False
            #     self._detected_bottle_index = None
            #     self._bottle_found_location = None
            #     self._external_target = False
            #
            #     self._found_bottles = []
            #
            #     self._total_run_count = 0
            #
            #     self._robots[0][0]._target_cell_index = 0
            #     self._robots[0][0]._target_cell = 6
            #     self._robots[0][0]._target_move_index = 0
            #     self._robots[0][0].init_move_data()
            #     self._robots[0][0].set_target(self._robots[0][0]._target_point)
            #
            #     for (obstacle, beer_brand) in world.generate_bottles(count=args.bottle_count):
            #         self.add_obstacle(obstacle, bottle=True, brand=beer_brand)
            #
            #     self._finished_simulation_count += 1
            #     self._text_file = open("./prints/" + args.input_file + "/" + str(self._finished_simulation_count) + "/" + "monitoring_summary" + str(self._total_run_count) + ".txt", "w")

    @staticmethod
    def generate_position():
        return tuple(np.random.uniform(1.5, 1.5, 2))

    @staticmethod
    def set_safe_position(robot_data):
        while True:
            robot_data.x, robot_data.y = 22.5, -12.5
            robot_data.theta = np.random.uniform(0, 2 * np.pi)
            if min(robot_data.sensors[0].distances) >= 0.30:
                return

    def reset(self, robot_data, frame_name):
        self.set_safe_position(robot_data)
        self._update_moving_object(robot_data, frame_name)


def get_args():
    """Gets parsed command-line arguments.

    Returns:
        Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(description="avoids obstacles")
    parser.add_argument("--input-file", default="input_file_6", type=str, help="input file name")
    parser.add_argument("--exploration-radius", default=5.0, type=float,
                        help="radius for detecting targets")
    parser.add_argument("--bottle-count", default=10, type=int,
                        help="number of bottles in area")
    parser.add_argument("--simulation-number", default=0, type=int,
                        help="number of bottles in area")
    parser.add_argument("--show-covered-area", default=False,
                        help="display covered area on map")
    parser.add_argument("--boust", action="store_true", default=False,
                        help="boust decomposıtıon",
                        dest="boust")
    parser.add_argument("--no-display", action="store_false", default=True,
                        help="whether to display the simulator or not",
                        dest="display")

    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()

    decomposed_map = VerticalCellDecomposition().process_input_file(args.input_file, args.boust)

    world = World(decomposed_map["boundaries"]["horizontal"], decomposed_map["boundaries"]["vertical"],
                  cells=decomposed_map["cell_points"])
    sim = Simulator(world)
    for obstacle in decomposed_map["obstacle_edges"]:
        for index in range(len(obstacle)):
            obstacle_vertices = []
            vertice = obstacle[index]

            # Add one side of polygon
            if index != len(obstacle) - 1:
                next_vertice = obstacle[index + 1]
            else:
                next_vertice = obstacle[0]

            obstacle_vertices.append(
                [np.float64(vertice[0] - world._width / 2.0), np.float64(vertice[1] - world._height / 2.0), 0.0])
            obstacle_vertices.append(
                [np.float64(next_vertice[0] - world._width / 2.0), np.float64(next_vertice[1] - world._height / 2.0),
                 0.0])
            obstacle_vertices.append(
                [np.float64(next_vertice[0] - world._width / 2.0), np.float64(next_vertice[1] - world._height / 2.0),
                 10.0])
            obstacle_vertices.append(
                [np.float64(vertice[0] - world._width / 2.0), np.float64(vertice[1] - world._height / 2.0), 10.0])

            obstacle_object = Obstacle(obstacle_vertices)
            sim.add_obstacle(obstacle_object, color=[0.93, 1., 1.])



    new_path = "./prints/" + args.input_file + "/bottle_locations.txt"
    if os.path.exists(new_path):
        file_handler = codecs.open(new_path, "r", encoding="utf-8")
        raw_data = file_handler.read()
        raw_data = raw_data.split("\n")
        for line in raw_data:
            if line != "":
                bottle_data = line.split(",")
                sim.add_obstacle(world.generate_bottle(float(bottle_data[1]), float(bottle_data[2])), bottle=True, brand=bottle_data[0])
    else:
        for (obstacle, beer_brand) in world.generate_bottles(count=args.bottle_count):
            sim.add_obstacle(obstacle, bottle=True, brand=beer_brand)

    moveData = {}
    for cell_id in decomposed_map["cells"]:
        cell = decomposed_map["cells"][cell_id]
        moveData[cell_id] = []
        for path in cell:
            moveData[cell_id].append([path[0] - world._width / 2.0, path[1] - world._height / 2.0, 0.0])

    sim.update_locator()

    robot = Drone(move_data=moveData, path=decomposed_map["path"], altitude=args.exploration_radius / 2)
    robot.attach_sensor(RaySensor(z_distance=args.exploration_radius / 2, bottles=sim.bottle_locations))


    new_path = "./prints/" + args.input_file + "/emergency.txt"
    if os.path.exists(new_path):
        file_handler = codecs.open(new_path, "r", encoding="utf-8")
        raw_data = file_handler.read()
        raw_data = raw_data.split("\n")
        robot._target_cell_index = int(raw_data[0])
        robot._target_move_index = int(raw_data[1])
        robot.init_move_data()

    new_target = robot._target_point
    robot.set_target(new_target)
    sim.add_target(new_target)

    sim.set_safe_position(robot)
    sim.add_robot(robot)

    sim.run(args.display, args.simulation_number)
