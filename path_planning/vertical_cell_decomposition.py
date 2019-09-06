# Complete coverage of field with obstacles using Vertical Cell Decomposition, Reeb Graph, Euler Tour

import codecs
import os
from collections import OrderedDict

import matplotlib.pyplot as plt

from helpers.decomposition import *
from helpers.euler_tour import *
from helpers.geometry import *
from helpers.graph import *
from helpers.maptools import *


class VerticalCellDecomposition(object):
    """Vertical Cell Decomposition."""

    def __init__(self):
        """Constructs variables.
        """
        self._input_file = ""

        self._boundary = []
        self._obstacles = []
        self._quad_cells = []

        self._obstacle_edges = {}
        self._simple_obstacle_edges = {}
        self._obstacle_limits = {}
        self._all_vertices = {}

        self.results_by_cells = {}
        self.path_by_cells = []

        self.dpi = 600

    def load_raw_data(self):
        """
        Load raw map data from specified file
        """
        dirname = os.path.dirname(__file__)
        filename = dirname + "/maps/" + self._input_file
        file_handler = codecs.open(filename, "r", encoding="utf-8")
        raw_data = file_handler.read()
        raw_data = raw_data.split("\n")
        if len(raw_data) < 2:
            print("Incorrect format of the input file")

        return raw_data

    def parse_input_line(self, line):
        """
        Parse line from input file.
        Input line should start with '(' and end with ')'

        :type line: string
        """
        temp2 = []
        line = [i.strip() for i in line.split(",")]
        vertex = []
        for index, i in enumerate(line):
            if i[0] == "(":
                i = i[1:]
            if i[len(i) - 1] == ")":
                i = i[:-1]
            vertex.append(int(i))
            if index % 2 != 0:
                temp2.append(vertex)
                vertex = []

        return temp2

    def draw_problem(self):
        """
        Create a plot and add map data to plot.
        """
        bnd_x = [i.x for i in self._boundary]
        bnd_x.append(self._boundary[0].x)
        bnd_y = [i.y for i in self._boundary]
        bnd_y.append(self._boundary[0].y)
        poly_x = []
        poly_y = []

        # Draw the boundary
        plt.plot(bnd_x, bnd_y)

        for index, i in enumerate(self._obstacles):
            poly_x.append([p[0] for p in i])
            poly_y.append([p[1] for p in i])
            # polygon draw
            plt.fill(poly_x[index], poly_y[index], color="#01579b")

        # for index, vertex in enumerate(self._obstacles[0]):
        #     import string
        #     plt.plot(vertex[0], vertex[1], 'ko')
        #     if index < 2:
        #         plt.text(vertex[0] - 6, vertex[1] - 22, string.ascii_uppercase[index], fontsize=12)
        #     elif index == 3:
        #         plt.text(vertex[0] - 6, vertex[1] + 6, string.ascii_uppercase[index], fontsize=12)
        #     elif index == 2:
        #         plt.text(vertex[0] + 6, vertex[1] - 6, string.ascii_uppercase[index], fontsize=12)
        #     else:
        #         plt.text(vertex[0] - 12, vertex[1] + 6, string.ascii_uppercase[index], fontsize=12)

        plt.savefig("./prints/" + self._input_file + "/map.png", dpi=self.dpi)

    def is_edge_breaks_object(self, vertice, other_vertice, from_obstacle=-1, from_obstacle_other=-1):
        """
        Check if edge with given vertices breaks through object(s).

        :param vertice:
        :param other_vertice:
        :param from_obstacle:
        :param from_obstacle_other:
        :return:
        :rtype: boolean
        """
        object_indexs = {}
        if vertice.obstacle != -1:
            object_indexs[vertice.obstacle] = True

        if other_vertice.obstacle != -1:
            object_indexs[other_vertice.obstacle] = True

        if from_obstacle != -1:
            object_indexs[from_obstacle] = True

        if from_obstacle_other != -1:
            object_indexs[from_obstacle_other] = True

        if len(object_indexs) == 1:
            check_list = [vertice, other_vertice]
            selected_object_index = list(object_indexs.keys())[0]
            if [vertice.x, vertice.y, selected_object_index] in self._obstacles[selected_object_index]:
                check_list.pop(0)
            if [other_vertice.x, other_vertice.y, selected_object_index] in self._obstacles[selected_object_index]:
                check_list.pop(0)

            if len(check_list) == 0:
                if [[vertice.x, vertice.y], [other_vertice.x, other_vertice.y]] in self._simple_obstacle_edges[
                    selected_object_index]:
                    return False
            else:
                for edge in self._simple_obstacle_edges[selected_object_index]:
                    if edge[0].x < edge[1].x:
                        range_x = range(int(edge[0].x), int(edge[1].x) + 1)
                    else:
                        range_x = range(int(edge[1].x), int(edge[0].x) + 1)
                    if edge[0].y < edge[1].y:
                        range_y = range(int(edge[0].y), int(edge[1].y) + 1)
                    else:
                        range_y = range(int(edge[1].y), int(edge[0].y) + 1)
                    if vertice.x in range_x and vertice.y in range_y and other_vertice.x in range_x and other_vertice.y in range_y:
                        return False

        for object_index in object_indexs:
            if object_index >= 0:
                poly_obstacle = Polygon([x[:2] for x in self._obstacles[object_index]])
                diff = np.abs(vertice.x - other_vertice.x)
                start = other_vertice
                slope = (vertice.y - other_vertice.y) / ((vertice.x - other_vertice.x) * 1.0)
                if np.abs(slope) > 5:
                    diff = np.abs(vertice.y - other_vertice.y)
                if other_vertice.x > vertice.x:
                    start = vertice

                for index in range(1, int(diff + 1)):
                    if slope > 5:
                        point_floor = Point([np.floor(start.x + index / slope), start.y + index])
                        point_ceil = Point([np.ceil(start.x + index / slope), start.y + index])
                    elif slope < -5:
                        point_floor = Point([np.floor(start.x - index / slope), start.y - index])
                        point_ceil = Point([np.ceil(start.x - index / slope), start.y - index])
                    else:
                        point_floor = Point([start.x + index, np.floor(start.y + index * slope)])
                        point_ceil = Point([start.x + index, np.ceil(start.y + index * slope)])
                    if poly_obstacle.contains(point_ceil) and poly_obstacle.contains(point_floor):
                        return True

        return False

    def is_cells_collapse(self, cell, next_cell):
        """
        Check if two cells collapses each other. Return number of vertices contained by two cells.

        :param cell:
        :param next_cell:
        :return:
        """
        cell_poly = Polygon([[x.x, x.y] for x in next_cell])
        total_contain = 0
        for vertice in cell:
            if cell_poly.covers(Point(vertice.x, vertice.y)):
                total_contain += 1

        return total_contain

    def find_cells(self, lines_and_obstacles):
        """
        Find cells from given linebreaks.
        lines_and_obstacles dict have information for each vertical line for sub-cells. Vertical lines highlight points
        for cell bounds which are detected by ertice information of obstacles.
        
        Each vertical line have information of obstacles intersections. For a certain x value there can be multiple 
        obstacle intersection. And each obstacle can have multiple intersection points. These intersection points can be
        either vertice points or points from edges.
        
        Algorithm is trying to create biggest possible sub-cell(s) with given intersection points. Each vertice is 
        triying to match with 3 different intersection points. Imaginary line between matched points should not break 
        into any obstacle.
        
        Algorithm calculates all possible cells and then removes duplicated and collapsed cells.

        :type lines_and_obstacles: 
        """
        cells = []
        added_cells_start = []
        added_cells_total = {}
        for line_loop, current_x in enumerate(sorted(lines_and_obstacles)):
            if line_loop < len(lines_and_obstacles) - 1:
                line = lines_and_obstacles[current_x]
                for loop, obstacle_index in enumerate(line):
                    current_obstacle = line[obstacle_index]
                    for vertice_index in current_obstacle:
                        vertice = current_obstacle[vertice_index]
                        matched_vertice = None
                        sorted_keys = sorted(lines_and_obstacles)
                        expected_next_x = sorted_keys[line_loop + 1]
                        for next_x in sorted(lines_and_obstacles)[line_loop + 1:]:
                            next_line = lines_and_obstacles[next_x]
                            for next_line_obstacle_index in next_line:
                                if expected_next_x == next_x or next_line_obstacle_index == obstacle_index:
                                    for next_line_vertice_index in next_line[next_line_obstacle_index]:
                                        next_line_vertice = next_line[next_line_obstacle_index][next_line_vertice_index]
                                        from_obstacle_id = -1
                                        next_vertice_from_obstacle_id = -1
                                        if "from-obstacle" in vertice.keys():
                                            from_obstacle_id = vertice["from-obstacle"]
                                        else:
                                            from_obstacle_id = vertice["point"].obstacle
                                        if "from-obstacle" in next_line_vertice.keys():
                                            next_vertice_from_obstacle_id = next_line_vertice["from-obstacle"]
                                        else:
                                            next_vertice_from_obstacle_id = next_line_vertice["point"].obstacle

                                        if "all-line" in vertice:
                                            if next_x != expected_next_x:
                                                break
                                            else:
                                                if not (self.is_edge_breaks_object(vertice["start"],
                                                                                   next_line_vertice["end"],
                                                                                   from_obstacle=from_obstacle_id,
                                                                                   from_obstacle_other=next_vertice_from_obstacle_id) or \
                                                        self.is_edge_breaks_object(vertice["end"],
                                                                                   next_line_vertice["start"],
                                                                                   from_obstacle=from_obstacle_id,
                                                                                   from_obstacle_other=next_vertice_from_obstacle_id)):
                                                    matched_vertice = next_line_vertice
                                                continue

                                        if "all-line" in next_line_vertice:
                                            if next_x == expected_next_x:
                                                if not (self.is_edge_breaks_object(vertice["start"],
                                                                                   next_line_vertice["end"],
                                                                                   from_obstacle=from_obstacle_id,
                                                                                   from_obstacle_other=next_vertice_from_obstacle_id) or \
                                                        self.is_edge_breaks_object(vertice["end"],
                                                                                   next_line_vertice["start"],
                                                                                   from_obstacle=from_obstacle_id,
                                                                                   from_obstacle_other=next_vertice_from_obstacle_id)):
                                                    matched_vertice = next_line_vertice
                                                    break
                                            continue

                                        if not self.is_edge_breaks_object(vertice["point"], next_line_vertice["point"],
                                                                          from_obstacle=from_obstacle_id,
                                                                          from_obstacle_other=next_vertice_from_obstacle_id):
                                            if matched_vertice is not None:
                                                if matched_vertice["start"].y >= next_line_vertice["start"].y:
                                                    matched_vertice = next_line_vertice
                                            else:
                                                matched_vertice = next_line_vertice
                                        elif matched_vertice is not None:
                                            break
                            if matched_vertice is not None:
                                next_line_start = matched_vertice["start"]
                                line_start = vertice["start"]

                                from_obstacle_id = -1
                                next_vertice_from_obstacle_id = -1
                                if "from-obstacle" in vertice.keys():
                                    from_obstacle_id = vertice["from-obstacle"]
                                if "from-obstacle" in matched_vertice.keys():
                                    next_vertice_from_obstacle_id = matched_vertice["from-obstacle"]

                                if self.is_edge_breaks_object(matched_vertice["start"], vertice["start"],
                                                              from_obstacle=from_obstacle_id,
                                                              from_obstacle_other=next_vertice_from_obstacle_id):
                                    if self.is_edge_breaks_object(matched_vertice["start"], vertice["point"],
                                                                  from_obstacle=from_obstacle_id,
                                                                  from_obstacle_other=next_vertice_from_obstacle_id):
                                        if self.is_edge_breaks_object(matched_vertice["point"], vertice["start"],
                                                                      from_obstacle=from_obstacle_id,
                                                                      from_obstacle_other=next_vertice_from_obstacle_id):
                                            if self.is_edge_breaks_object(matched_vertice["point"], vertice["point"],
                                                                          from_obstacle=from_obstacle_id,
                                                                          from_obstacle_other=next_vertice_from_obstacle_id):
                                                continue
                                            else:
                                                next_line_start = matched_vertice["point"]
                                                line_start = vertice["point"]
                                        else:
                                            next_line_start = matched_vertice["point"]
                                    else:
                                        line_start = vertice["point"]

                                if self.is_edge_breaks_object(matched_vertice["end"], vertice["end"],
                                                              from_obstacle=from_obstacle_id,
                                                              from_obstacle_other=next_vertice_from_obstacle_id):
                                    continue

                                cell = [
                                    line_start,
                                    next_line_start,
                                    matched_vertice["end"],
                                    vertice["end"]
                                ]

                                cell_start = [vertice["start"].x, next_line_start.x]

                                if not cell[1].equals(cell[2]):
                                    if cell_start in added_cells_start:
                                        cell_index = added_cells_start.index(cell_start)
                                        total_contain = 0
                                        deletion_index = None
                                        for index_x, earlier_cell in enumerate(added_cells_total[cell_index]):
                                            contain_count = self.is_cells_collapse(earlier_cell, cell)
                                            if contain_count > 2:
                                                total_contain = contain_count
                                                deletion_index = earlier_cell

                                        if total_contain == 4:
                                            cells.append(cell)
                                            cells.remove(deletion_index)
                                            added_cells_total[cell_index].append(cell)
                                        elif total_contain == 0:
                                            cells.append(cell)
                                            added_cells_total[cell_index].append(cell)
                                    else:
                                        added_cells_start.append(cell_start)
                                        cells.append(cell)
                                        added_cells_total[len(added_cells_start) - 1] = [cell]

                                break

        return cells

    def sort_cells(self, source_cells):
        """
        Nearest neighbor approach for cell sorting.

        :param source_cells: 
        :return: 
        """
        new_cells = []
        cells = deepcopy(source_cells)

        origin = [0, 0]
        total_cells = len(cells)
        while len(new_cells) < total_cells:
            shortest = []
            for cell in cells:
                shortest.append(
                    np.sqrt(np.square(np.abs(cell[0].x - origin[0])) + np.square(np.abs(cell[0].y - origin[1]))))

            min_index = shortest.index(min(shortest))
            new_cells.append(cells[min_index])
            origin = [cells[min_index][0].x, cells[min_index][0].y]
            cells.pop(min_index)

        return new_cells

    def draw_cells(self):
        """
        Draw cells to plot.
        """
        for index, i in enumerate(self._quad_cells):
            x = [j.x for j in i]
            y = [j.y for j in i]
            plt.plot(x, y)
            # plt.text(np.mean(x) - 5, np.mean(y) - 5, index, fontsize=8, color="#ffffff")
            plt.text(np.mean(x) - 7, np.mean(y)-5, str(index), fontsize=11)

    def calculate_graph(self, plt):
        """
        Calculate reeb graph of given map.
        
        :param plt: 
        :return: 
        """
        # ----------------------------------------------------------------------
        # Get the graph
        graph_vertices = []
        graph_edges = []
        graph_vertices_cell_connection = {}

        for index1 in range(len(self._quad_cells)):
            same_boundary = []
            for index2 in range(len(self._quad_cells)):
                if (index1 != index2):
                    if self._quad_cells[index1][1].x == self._quad_cells[index2][0].x:
                        for y_left in range(int(self._quad_cells[index1][1].y), int(self._quad_cells[index1][2].y)):
                            if y_left in range(self._quad_cells[index2][0].y, self._quad_cells[index2][3].y):
                                same_boundary.append(index2)
                                break

            temp = self._quad_cells[index1][0:4]
            centroid_vertex = centroid(temp)
            place = centroid_vertex.find_point(graph_vertices)
            if (place == -1):
                graph_vertices.append(centroid_vertex)

            if (len(same_boundary) == 1):
                temp_edge_middle = centroid([self._quad_cells[index1][1], self._quad_cells[index1][2]])
                graph_vertices.append(temp_edge_middle)
                n = len(graph_vertices) - 1
                if (place != -1):
                    graph_edges.append([place, n])
                    graph_vertices_cell_connection[place] = index1
                else:
                    graph_edges.append([n - 1, n])
                    graph_vertices_cell_connection[n - 1] = index1
                temp = self._quad_cells[same_boundary[0]][0:4]
                curr_centroid_vertex = centroid(temp)
                place2 = curr_centroid_vertex.find_point(graph_vertices)
                if (place2 == -1):
                    graph_vertices.append(curr_centroid_vertex)
                    graph_edges.append([n, n + 1])
                else:
                    graph_edges.append([n, place2])

            elif (len(same_boundary) > 1):
                n = len(graph_vertices) - 1
                if (place != -1):
                    use = place
                else:
                    use = n
                for index, i in enumerate(same_boundary):
                    temp = self._quad_cells[i][0:4]
                    curr_centroid_vertex = centroid(temp)
                    temp_edge_middle = centroid([self._quad_cells[i][0], self._quad_cells[i][3]])
                    graph_vertices.append(temp_edge_middle)
                    pl1 = len(graph_vertices) - 1
                    hmmm = curr_centroid_vertex.find_point(graph_vertices)
                    if (hmmm == -1):
                        graph_vertices.append(curr_centroid_vertex)
                        pl2 = len(graph_vertices) - 1
                    else:
                        pl2 = hmmm
                    graph_edges.append([use, pl1])
                    graph_edges.append([pl1, pl2])
                    graph_vertices_cell_connection[use] = index1

            else:
                graph_vertices_cell_connection[place] = index1

        # Convert graph in adjacency list format
        graph = []
        for j in range(len(graph_vertices)):
            graph.append([])
            for i in graph_edges:
                if (i[0] == j):
                    graph[j].append(i[1])
                elif (i[1] == j):
                    graph[j].append(i[0])

        path = bfs(graph, len(graph_vertices) - 2, len(graph_vertices) - 1)

        if (path is None):
            print("No path found. Sorry")
        else:
            print("Path found.")

        # plt.plot(i.x,i.y, marker="x")

        graph_edges_without_bounds = []
        for index, i in enumerate(graph_edges):
            if index % 2 == 0:
                graph_edges_without_bounds.append([i[0], graph_edges[index + 1][1]])

        return graph_edges_without_bounds, graph_vertices, graph_vertices_cell_connection

    def find_cell_lines(self):
        """
        Find vertical lines and obstacle intersections of each line.

        :return:
        """
        lines_and_obstacles = {}
        for current_x in sorted(self._all_vertices):
            lines_and_obstacles[current_x] = {}
            previous_vertice_id = []
            for vertice_id, vertice in enumerate(sorted(self._all_vertices[current_x], key=lambda x: x.y)):
                obstacle_id = vertice.obstacle
                if not obstacle_id in lines_and_obstacles[current_x]:
                    lines_and_obstacles[current_x][obstacle_id] = {
                        vertice_id: {"point": vertice, "start": None, "end": None}}
                else:
                    lines_and_obstacles[current_x][obstacle_id][vertice_id] = {"point": vertice, "start": None,
                                                                               "end": None}

                self_intersection = {"bottom": [], "top": []}
                for edge in self._obstacle_edges[obstacle_id]:
                    if current_x in range(edge[0][0] - 1, edge[1][0] + 1):
                        res_bottom = segment_intersection(point(current_x, 0), vertice, point(edge[0][0], edge[0][1]),
                                                          point(edge[1][0], edge[1][1]))
                        res_top = segment_intersection(vertice, point(current_x, self._boundary[2].y),
                                                       point(edge[0][0], edge[0][1]),
                                                       point(edge[1][0], edge[1][1]))

                        if res_bottom != -1:
                            if res_bottom.y in range(-1, vertice.y):
                                if res_bottom.y != vertice.y:
                                    self_intersection["bottom"].append(res_bottom.y)
                                    self_intersection["bottom"].sort()
                        if res_top != -1:
                            if res_top.y in range(vertice.y, self._boundary[2].y + 1):
                                if res_top.y != vertice.y:
                                    self_intersection["top"].append(res_top.y)
                                    self_intersection["top"].sort()

                two_vertice_on_same_line = False
                if len(self_intersection["bottom"]) > 0:
                    if [[current_x, vertice.y, obstacle_id],
                        [current_x, self_intersection["bottom"][-1], obstacle_id]] in \
                            self._obstacle_edges[obstacle_id]:
                        previous_vertice_id = [index for index, x in
                                               enumerate(sorted(self._all_vertices[current_x], key=lambda x: x.y)) if
                                               x.equals(point(current_x, self_intersection["bottom"][-1], obstacle_id))]

                        if self._obstacle_limits[obstacle_id]["x_low"] == vertice.x or \
                                self._obstacle_limits[obstacle_id][
                                    "x_high"] == vertice.x:
                            lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] = vertice
                            two_vertice_on_same_line = True

                        elif previous_vertice_id[0] < vertice_id:
                            lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] = \
                                lines_and_obstacles[current_x][obstacle_id][previous_vertice_id[0]]["start"]
                            previous_vertice_id = []
                    elif not Polygon([x[:2] for x in self._obstacles[obstacle_id]]).contains(
                            Point(current_x, (vertice.y + self_intersection["bottom"][-1]) / 2)):
                        obstacle_index_for_point = -1
                        if [current_x, self_intersection["bottom"][-1], obstacle_id] in self._obstacles[obstacle_id]:
                            obstacle_index_for_point = obstacle_id
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] = point(current_x,
                                                                                                 self_intersection[
                                                                                                     "bottom"][
                                                                                                     -1],
                                                                                                 obstacle_index_for_point)
                    else:
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] = vertice
                if len(self_intersection["top"]) > 0:
                    lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"] = vertice

                intersections = [[0, -1], [self._boundary[2].y, -1]]
                intersections_only_y = [0, self._boundary[2].y]
                if lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"] is None or \
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] is None:
                    for inter_obstacle_id in self._obstacle_edges:
                        if inter_obstacle_id != obstacle_id:
                            for edge in self._obstacle_edges[inter_obstacle_id]:
                                if current_x in range(edge[0][0] - 1, edge[1][0] + 1):
                                    res_all = segment_intersection(point(current_x, 0),
                                                                   point(current_x, self._boundary[2].y),
                                                                   point(edge[0][0], edge[0][1]),
                                                                   point(edge[1][0], edge[1][1]))
                                    if res_all != -1:
                                        intersections.append([res_all.y, inter_obstacle_id])
                                        intersections_only_y.append(res_all.y)
                                        intersections.sort()

                if lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"] is None:
                    near_point = [y for y in intersections if y[0] >= vertice.y][0]
                    lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"] = point(current_x, near_point[0],
                                                                                           near_point[1])
                if lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] is None:
                    near_point = [y for y in intersections if y[0] <= vertice.y][-1]
                    lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"] = point(current_x, near_point[0],
                                                                                             near_point[1])

                if two_vertice_on_same_line:
                    first_previous_vertice_id = [index for index, x in
                                                 enumerate(sorted(self._all_vertices[current_x], key=lambda x: x.y)) if
                                                 x.equals(
                                                     point(current_x, self_intersection["bottom"][0], obstacle_id))]

                    top_bound = lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"].y
                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 10] = {
                        "start": lines_and_obstacles[current_x][obstacle_id][first_previous_vertice_id[0]]["start"],
                        "point": point(current_x,
                                       (vertice.y + self._all_vertices[current_x][previous_vertice_id[0]].y) / 2,
                                       obstacle_id),
                        "end": point(current_x, top_bound, obstacle_id)
                    }

                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 10]["from-obstacle"] = obstacle_id
                    previous_vertice_id = []

                if len(previous_vertice_id) > 0 and previous_vertice_id[0] == vertice_id:
                    lines_and_obstacles[current_x][obstacle_id][vertice_id - 1]["start"] = \
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"]
                    previous_vertice_id = []

                if lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"].y in intersections_only_y and \
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"].y in intersections_only_y and \
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"].y != \
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"].y:

                    upper_y = deepcopy(lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"].y)
                    upper_y_obstacle = deepcopy(lines_and_obstacles[current_x][obstacle_id][vertice_id]["end"].obstacle)
                    lower_y = deepcopy(lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"].y)
                    lower_y_obstacle = deepcopy(
                        lines_and_obstacles[current_x][obstacle_id][vertice_id]["start"].obstacle)

                    if (vertice_id + 10) in lines_and_obstacles[current_x][obstacle_id]:
                        upper_y = deepcopy(lines_and_obstacles[current_x][obstacle_id][vertice_id + 10]["end"].y)
                        upper_y_obstacle = deepcopy(
                            lines_and_obstacles[current_x][obstacle_id][vertice_id + 10]["end"].obstacle)
                        lower_y = deepcopy(lines_and_obstacles[current_x][obstacle_id][vertice_id + 10]["start"].y)
                        lower_y_obstacle = deepcopy(
                            lines_and_obstacles[current_x][obstacle_id][vertice_id + 10]["start"].obstacle)

                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 1] = {
                        "start": deepcopy(vertice),
                        "point": deepcopy(vertice),
                        "end": point(current_x, upper_y, upper_y_obstacle)
                    }
                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 1]["point"].y = (vertice.y + upper_y) / 2
                    if lines_and_obstacles[current_x][obstacle_id][vertice_id + 1][
                        "point"].obstacle != upper_y_obstacle:
                        lines_and_obstacles[current_x][obstacle_id][vertice_id + 1]["point"].obstacle = -1
                        lines_and_obstacles[current_x][obstacle_id][vertice_id + 1]["from-obstacle"] = obstacle_id

                    lines_and_obstacles[current_x][obstacle_id][vertice_id] = {
                        "start": point(current_x, lower_y, lower_y_obstacle),
                        "point": deepcopy(vertice),
                        "end": deepcopy(vertice)
                    }
                    lines_and_obstacles[current_x][obstacle_id][vertice_id]["point"].y = lower_y + (
                            vertice.y - lower_y) / 2

                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 2] = {
                        "start": point(current_x, lower_y, lower_y_obstacle),
                        "point": point(current_x, upper_y, upper_y_obstacle),
                        "end": point(current_x, upper_y, upper_y_obstacle)
                    }
                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 2]["all-line"] = True
                    lines_and_obstacles[current_x][obstacle_id][vertice_id + 2]["from-obstacle"] = obstacle_id

            for obstacle_index in lines_and_obstacles[current_x]:
                lines_and_obstacles[current_x][obstacle_index] = OrderedDict(
                    sorted(lines_and_obstacles[current_x][obstacle_index].items(), key=lambda x: x[1]["point"].y))
        return lines_and_obstacles

    def process_input_file(self, file_name, boust):
        """
        Get map information from given file and calculate vertical cell decomposition for given map.

        :param file_name:
        :return:
        """
        # dpi level of saved graph - Higher for high resolution output--
        # True - euler tour graph will rendered to file
        print_graph = False

        new_path = "./prints/" + "input_file_6" + "/bottle_locations.txt"
        if os.path.exists(new_path):
            file_handler = codecs.open(new_path, "r", encoding="utf-8")
            raw_data = file_handler.read()
            raw_data = raw_data.split("\n")
            for line in raw_data:
                bottle_data = line.split(",")

        self._input_file = file_name
        raw_data = self.load_raw_data()
        new_path = "./prints/" + self._input_file
        if not os.path.exists(new_path):
            os.makedirs(new_path)

        # Extract vertices----------------------------------------------
        temp = self.parse_input_line(raw_data[0])
        self._boundary = [point(i[0], i[1]) for i in temp]

        # Extract obstacles
        for i in raw_data[1:len(raw_data) - 1]:
            self._obstacles.append(self.parse_input_line(i))

        obstacle_vertices_as_points = []
        # sort by x-values
        for index, i in enumerate(self._obstacles):
            for j in i:
                j.append(index)
                temp = point(j[0], j[1], j[2])
                obstacle_vertices_as_points.append(temp)

        # Draw the problem
        self.draw_problem()

        # -----------------------------------------------------------
        # Find and set obstacle vertices and obstacle edges
        for index, i in enumerate(self._obstacles):
            self._obstacle_edges[index] = []
            self._simple_obstacle_edges[index] = []
            self._obstacle_limits[index] = {"x_low": 1000000, "x_high": -1}
            for vertex_count, j in enumerate(i):
                if j[0] in self._all_vertices:
                    self._all_vertices[j[0]].append(point(j[0], j[1], index))
                else:
                    self._all_vertices[j[0]] = [point(j[0], j[1], index)]

                # set obstacle edges
                vertex = deepcopy(j)
                if vertex_count > 0:
                    edge_start = deepcopy(i[vertex_count - 1])
                    self._obstacle_edges[index].append([edge_start, vertex])
                    self._obstacle_edges[index].append([vertex, edge_start])
                    self._simple_obstacle_edges[index].append(
                        [Point(edge_start[0], edge_start[1]), Point(vertex[0], vertex[1])])
                if vertex_count == len(i) - 1:
                    edge_start = deepcopy(i[0])
                    self._obstacle_edges[index].append([edge_start, vertex])
                    self._obstacle_edges[index].append([vertex, edge_start])
                    self._simple_obstacle_edges[index].append(
                        [Point(edge_start[0], edge_start[1]), Point(vertex[0], vertex[1])])

                if self._obstacle_limits[index]["x_low"] > j[0]:
                    self._obstacle_limits[index]["x_low"] = j[0]
                if self._obstacle_limits[index]["x_high"] < j[0]:
                    self._obstacle_limits[index]["x_high"] = j[0]

        # -----------------------------------------------------------
        # find and set obstacle vertices and obstacle edges
        lines_and_obstacles = self.find_cell_lines()

        # -----------------------------------------------------------
        # Find vertical limits of env

        y_limit_lower = self._boundary[0].y
        y_limit_upper = self._boundary[2].y

        # ------------------------------------------------------
        # Find Polygon cells naively.

        lines_and_obstacles = OrderedDict(sorted(lines_and_obstacles.items()))

        cells = self.find_cells(lines_and_obstacles)

        cells = self.sort_cells(cells)
        if boust:
            cells[1] = [
                cells[1][0],
                cells[2][0],
                cells[3][0],
                cells[4][0],
                cells[5][0],
                cells[5][1],
                cells[5][2],
                cells[5][3],
                cells[4][3],
                cells[3][3],
                cells[2][3],
                cells[1][2],
                cells[1][3],
            ]
            cells.pop(2)
            cells.pop(2)
            cells.pop(2)
            cells.pop(2)
            cells[6] = [
                cells[6][0],
                cells[7][1],
                cells[7][2],
                cells[7][3],
                cells[6][2],
                cells[6][3]
            ]
            cells.pop(7)

        # -------------------------------------------------------
        # Merge overlapping Polygons
        self._quad_cells = cells

        # ------------------------------------------------------
        # Add boundary lines
        if self._boundary[0].x != obstacle_vertices_as_points[0].x:
            self._quad_cells.append(
                [self._boundary[0], point(obstacle_vertices_as_points[0].x, y_limit_lower),
                 point(obstacle_vertices_as_points[0].x, y_limit_upper),
                 self._boundary[3]])
        if self._boundary[1].x != obstacle_vertices_as_points[-1].x:
            self._quad_cells.append(
                [point(sorted(obstacle_vertices_as_points, key=lambda x: x.x)[-1].x, y_limit_lower),
                 self._boundary[1],
                 self._boundary[2],
                 point(sorted(obstacle_vertices_as_points, key=lambda x: x.x)[-1].x, y_limit_upper)])

        # -------------------------------------------------------
        # Plot final cells
        self._quad_cells = self.sort_cells(self._quad_cells)

        # print cells
        self.draw_cells()
        plt.savefig("./prints/" + self._input_file + "/map_with_cells.png", dpi=self.dpi)

        # ----------------------------------------------------------------------
        # Calculate and draw graph

        if os.path.exists("./maps/" + self._input_file + ".png"):
            fig, ax = plt.subplots()
            self.draw_cells()
            img = plt.imread("./maps/" + self._input_file + ".png")
            ax.imshow(img, origin='lower')
            plt.savefig("./prints/" + self._input_file + "/map_with_image.png", dpi=self.dpi)

        graph_edges, graph_vertices, graph_vertices_cell_connection = self.calculate_graph(plt)

        tour = find_euler_tour(graph_edges)
        visited_cells = {}
        last_visited = 0
        empty_connection = False

        text_file = open("./prints/" + self._input_file + "/tour_explanation.txt", "w")
        self._quad_cells = []
        for index, i in enumerate(cells):
            i.append(i[0])
            self._quad_cells.append(i)
            text_file.write("cell {0}: {1} {2} {3} {4}".format(index, i[0], i[1], i[2], i[3]))
            text_file.write("\n")

        if print_graph:
            # Draw everything--------------
            for index, i in enumerate(graph_vertices):
                plt.annotate(str(index), xy=(i.x, i.y), xytext=(i.x + 2, i.y - 2))

            # plt.plot(i.x,i.y, marker="x")

            for index, i in enumerate(graph_edges):
                temp_x = [graph_vertices[i[0]].x, graph_vertices[i[1]].x]
                temp_y = [graph_vertices[i[0]].y, graph_vertices[i[1]].y]
                plt.plot(temp_x, temp_y)
                # print graph
                plt.savefig("./prints/" + self._input_file + "/map_with_graph.png", dpi=self.dpi)
        # ----------------------------------------------------------------------
        # All points for decompose

        ax, _ = make_axis()
        colorMap = ["#e69500", "#808000", "#0e467d", "#ea8b9f",
                    "#224a72", "#e69500", "#808000", "#ea8b9f", "#c1ffc1", "#e69500", "#808000",
                    "#0d26c4",
                    "#3926a0",
                    "#65267c",
                    "#912658", "#0e467d", "#0e467d",
                    "#bd2634",
                    "#450c41",
                    "#61275d",
                    "#86b5d5",
                    "#2c7f84",
                    "#224a72", "#e69500", "#808000", "#0e467d", "#ea8b9f", "#c1ffc1", "#e69500", "#808000", "#0e467d",
                    "#224a72", "#e69500", "#808000", "#0e467d", "#ea8b9f", "#c1ffc1", "#e69500", "#808000", "#0e467d",
                    "#ea8b9f", "#c1ffc1"]

        # set origin
        rt = RotationTransform(66)
        origin = rotate_to(np.array([(1, 1)]), rt).tolist()
        result = None

        # create decomposed lines from polygons and draw lines to figure
        for idx, i in enumerate(self._quad_cells):

            if idx < len(self._quad_cells) - 2:
                if boust and idx == 1:
                    origin = [i[12].x, i[12].y]
                elif boust and idx == 6:
                    origin = [i[5].x, i[5].y]
                elif i[0].x < self._quad_cells[idx + 1][0].x:
                    origin = [self._quad_cells[idx][0].x, self._quad_cells[idx][0].y]
                    if result is not None:
                        if np.abs(result[-1][1] - self._quad_cells[idx][0].y) > \
                                np.abs(result[-1][1] - self._quad_cells[idx][3].y):
                            origin = [self._quad_cells[idx][3].x, self._quad_cells[idx][3].y]
                else:
                    origin = [self._quad_cells[idx][1].x, self._quad_cells[idx][1].y]
                    if result is not None:
                        if np.abs(result[-1][1] - self._quad_cells[idx][1].y) > \
                                np.abs(result[-1][1] - self._quad_cells[idx][2].y):
                            origin = [self._quad_cells[idx][2].x, self._quad_cells[idx][2].y]

            ext = []
            for j in i:
                ext.append((j.x, j.y))
            polygon = Polygon(ext)
            result = decompose(polygon, origin, width=5)
            self.results_by_cells[idx] = result
            self.path_by_cells.append(idx)

            text_file.write("\n")
            text_file.write("---Cell {0}---".format(idx))
            text_file.write("\n")

            result_length = len(result)

            for index in range(result_length - 1):
                if index > 1:
                    ll = LineString([result[index], result[index + 1]])
                    plot_line(ll, ax, color=colorMap[idx])

                    text_file.write(
                        "from ({0}, {1}) to ({2}, {3})".format(result[index][0], result[index][1], result[index + 1][0],
                                                               result[index + 1][1]))
                    text_file.write("\n")

        plt.savefig("./prints/" + self._input_file + "/map_decomposed.png", dpi=self.dpi)

        if len(tour) < len(self.path_by_cells):
            tour = [i*2 for i in self.path_by_cells]

        # for i in range(len(tour) - 1):
        #     first = tour[i]
        #     second = tour[i + 1]
        #     if not second in visited_cells:
        #         if empty_connection:
        #             first = last_visited
        #             empty_connection = False
        #
        #         print("from cell " + str(i) + " to cell " + str(
        #             i+1))
        #         text_file.write("from cell: {0} - to cell: {1}".format(i,
        #                                                                i+1))
        #         text_file.write("\n")
        #
        #         temp_x = [graph_vertices[first].x, graph_vertices[second].x]
        #         temp_y = [graph_vertices[first].y, graph_vertices[second].y]
        #         plt.plot(temp_x, temp_y, "--")
        #         visited_cells[second] = True
        #         last_visited = second
        #     else:
        #         empty_connection = True

        plt.savefig("./prints/" + self._input_file + "/map_with_euler_tour.png", dpi=self.dpi)

        text_file.close()

        print("Output written to file.. Drawing the result")

        # plt.show()

        return {
            "boundaries": {
                "horizontal": self._boundary[1].x - self._boundary[0].x,
                "vertical": self._boundary[2].y - self._boundary[1].y,
            },
            "obstacle_edges": self._obstacles,
            "path": self.path_by_cells,
            "cells": self.results_by_cells,
            "cell_points": self._quad_cells
        }


if __name__ == "__main__":
    VerticalCellDecomposition().process_input_file("input_file_6", False)
