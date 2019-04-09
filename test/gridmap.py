import numpy
import matplotlib.pyplot as plt
from utils import png_to_ogm
import numpy as np
import math
import math
from heapq import heappush, heappop
from utils import dist2d

class OccupancyGridMap:
    def __init__(self, data_array, cell_size, occupancy_threshold=0.2):
        """
        Creates a grid map
        :param data_array: a 2D array with a value of occupancy per cell (values from 0 - 1)
        :param cell_size: cell size in meters
        :param occupancy_threshold: A threshold to determine whether a cell is occupied or free.
        A cell is considered occupied if its value >= occupancy_threshold, free otherwise.
        """
        self.log_prob_map = data_array

        self.data = data_array
        self.dim_cells = data_array.shape
        self.dim_meters = (self.dim_cells[0] * cell_size, self.dim_cells[1] * cell_size)
        self.cell_size = cell_size
        self.occupancy_threshold = occupancy_threshold
        # 2D array to mark visited nodes (in the beginning, no node has been visited)
        self.visited = numpy.zeros(self.dim_cells, dtype=numpy.float32)
        self.path = []
        self.path_px = []

    def mark_visited_idx(self, point_idx):
        """
        Mark a point as visited.
        :param point_idx: a point (x, y) in data array
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.visited[y_index][x_index] = 1.0

    def mark_visited(self, point):
        """
        Mark a point as visited.
        :param point: a 2D point (x, y) in meters
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.mark_visited_idx((x_index, y_index))

    def is_visited_idx(self, point_idx):
        """
        Check whether the given point is visited.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is visited, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        if self.visited[y_index][x_index] == 1.0:
            return True
        else:
            return False

    def is_visited(self, point):
        """
        Check whether the given point is visited.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is visited, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_visited_idx((x_index, y_index))

    def get_data_idx(self, point_idx):
        """
        Get the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :return: the occupancy value of the given point
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        return self.log_prob_map[y_index][x_index]

    def get_data(self, point):
        """
        Get the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :return: the occupancy value of the given point
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.get_data_idx((x_index, y_index))

    def set_data_idx(self, point_idx, new_value):
        """
        Set the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :param new_value: the new occupancy values
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.log_prob_map[y_index][x_index] = new_value

    def set_data(self, point, new_value):
        """
        Set the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :param new_value: the new occupancy value
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        self.set_data_idx((x_index, y_index), new_value)

    def is_inside_idx(self, point_idx):
        """
        Check whether the given point is inside the map.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is inside the map, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            return False
        else:
            return True

    def is_inside(self, point):
        """
        Check whether the given point is inside the map.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is inside the map, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_inside_idx((x_index, y_index))

    def is_occupied_idx(self, point_idx):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is occupied, false otherwise
        """
        x_index, y_index = point_idx
        if self.get_data_idx((x_index, y_index)) >= self.occupancy_threshold:
            return True
        else:
            return False

    def is_occupied(self, point):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is occupied, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_occupied_idx((x_index, y_index))

    def get_index_from_coordinates(self, x, y):
        """
        Get the array indices of the given point.
        :param x: the point's x-coordinate in meters
        :param y: the point's y-coordinate in meters
        :return: the corresponding array indices as a (x, y) tuple
        """
        x_index = int(round(x/self.cell_size))
        y_index = int(round(y/self.cell_size))

        return x_index, y_index

    def get_coordinates_from_index(self, x_index, y_index):
        """
        Get the coordinates of the given array point in meters.
        :param x_index: the point's x index
        :param y_index: the point's y index
        :return: the corresponding point in meters as a (x, y) tuple
        """
        x = x_index*self.cell_size
        y = y_index*self.cell_size

        return x, y

    def update(self, robot, scan):
    
        for meas in scan:
                grid_map = self.occupancy_grid_map([robot.x, robot.y, robot.yaw+math.pi/2], meas)

    def bresenham(self, start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
     
        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
     
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
     
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
     
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
     
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
     
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
     
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
     
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def occupancy_grid_map(self, state, meas):
        # r - размер ячейки в метрах
        # позиция робота в метрах
    ##    print(state)
        if meas[1]/1000 < 4:
            x_robot = state[0]#m
            y_robot = state[1]#m
            angle_robot = state[2] #rad

            meas_dist = meas[1]/1000
        ##    print(meas_dist)
            angle_dist = self.normalize_angle(math.radians(meas[0]))
            # d - расстояние до препядствия 
         
            # позиция робота в индексах карты
            i_robot_pos = math.ceil(x_robot/self.cell_size)
            j_robot_pos = math.ceil(y_robot/self.cell_size)
            # позиция робота в индексах карты
            i_robot_pos = math.ceil(x_robot/self.cell_size)
            j_robot_pos = math.ceil(y_robot/self.cell_size)
            # позиция препятствия в метрах
            x_end_occ = x_robot + meas_dist* math.cos(angle_robot+angle_dist)
            y_end_occ = y_robot + meas_dist* math.sin(angle_robot+angle_dist)
            # позиция препятствия в индексах карты
            i_end_occ = math.ceil(x_end_occ/self.cell_size)
            j_end_occ = math.ceil(y_end_occ/self.cell_size)
            #начальная точка линии
            x1 = i_robot_pos
            y1 = j_robot_pos
            #конечная точка линии
            x2 = i_end_occ
            y2 = j_end_occ
        ##    bresenham(x1, y1, x2, y2)
            points = self.bresenham((x1, y1), (x2, y2))
            if -100 <= self.data[x2][y2] <= 100:
                self.data[x2][y2] += 0.6#np.log(0.65/0.35)# occupancy
##            m[x2][y2] = 1.0 - 1./(1.+np.exp(m[x2][y2]))
            for point in points[:-1]:
                i = point[0]
                j = point[1]
                if -100 <= self.data[i][j] <= 100:
                    self.data[i][j] += -0.2#np.log(0.35/0.65)# free
##                m[i][j] = 1.0 - 1./(1.+np.exp(m[i][j]))
                
    def update_path(self, start, goal):
        self.visited = numpy.zeros(self.dim_cells, dtype=numpy.float32)
        self.log_prob_map = 1.0 - 1./(1.+np.exp(self.data))
        self.path, self.path_px = self.a_star(start, goal, movement='8N')

    def _get_movements_4n(self):
        
        """
        Get all possible 4-connectivity movements.
        :return: list of movements with cost [(dx, dy, movement_cost)]
        """
        return [(1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0)]


    def _get_movements_8n(self):
        
        """
        Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1).
        :return: list of movements with cost [(dx, dy, movement_cost)]
        """
        s2 = math.sqrt(2)
        return [(1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0),
                (1, 1, s2),
                (-1, 1, s2),
                (-1, -1, s2),
                (1, -1, s2)]


    def a_star(self, start_m, goal_m, movement='8N', occupancy_cost_factor=10):
        
        """
        A* for 2D occupancy grid.

        :param start_m: start node (x, y) in meters
        :param goal_m: goal node (x, y) in meters
        :param gmap: the grid map
        :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
        :param occupancy_cost_factor: a number the will be multiplied by the occupancy probability
            of a grid map cell to give the additional movement cost to this cell (default: 3).

        :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
        """

        # get array indices of start and goal
        start = self.get_index_from_coordinates(start_m[0], start_m[1])
        goal = self.get_index_from_coordinates(goal_m[0], goal_m[1])

        # check if start and goal nodes correspond to free spaces
        if self.is_occupied_idx(start):
                print('Start node is not traversable')
            #raise Exception('Start node is not traversable')

        if self.is_occupied_idx(goal):
                print('Goal node is not traversable')
            #raise Exception('Goal node is not traversable')

        # add start node to front
        # front is a list of (total estimated cost to goal, total cost from start to node, node, previous node)
        start_node_cost = 0
        start_node_estimated_cost_to_goal = dist2d(start, goal) + start_node_cost
        front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None)]

        # use a dictionary to remember where we came from in order to reconstruct the path later on
        came_from = {}

        # get possible movements
        if movement == '4N':
            movements = self._get_movements_4n()
        elif movement == '8N':
            movements = self._get_movements_8n()
        else:
            raise ValueError('Unknown movement')

        # while there are elements to investigate in our front.
        while front:
            # get smallest item and remove from front.
            element = heappop(front)

            # if this has been visited already, skip it
            total_cost, cost, pos, previous = element
            if self.is_visited_idx(pos):
                continue

            # now it has been visited, mark with cost
            self.mark_visited_idx(pos)

            # set its previous node
            came_from[pos] = previous

            # if the goal has been reached, we are done!
            if pos == goal:
                break

            # check all neighbors
            for dx, dy, deltacost in movements:
                # determine new position
                new_x = pos[0] + dx
                new_y = pos[1] + dy
                new_pos = (new_x, new_y)

                # check whether new position is inside the map
                # if not, skip node
                if not self.is_inside_idx(new_pos):
                    continue

                # add node to front if it was not visited before and is not an obstacle
                if (not self.is_visited_idx(new_pos)) and (not self.is_occupied_idx(new_pos)):
                    potential_function_cost = self.get_data_idx(new_pos)*occupancy_cost_factor
                    new_cost = cost + deltacost + potential_function_cost
                    new_total_cost_to_goal = new_cost + dist2d(new_pos, goal) + potential_function_cost

                    heappush(front, (new_total_cost_to_goal, new_cost, new_pos, pos))

        # reconstruct path backwards (only if we reached the goal)
        path = []
        path_idx = []
        if pos == goal:
            while pos:
                path_idx.append(pos)
                # transform array indices to meters
                pos_m_x, pos_m_y = self.get_coordinates_from_index(pos[0], pos[1])
                path.append((pos_m_x, pos_m_y))
                pos = came_from[pos]

            # reverse so that path is from start to goal.
            path.reverse()
            path_idx.reverse()

        return path, path_idx

