import yaml
import math
from pathlib import Path
import shapely.geometry as geom
from shapely import affinity
import itertools
import random
from utils import *

# For Obstacles
import numpy as np
from scipy.spatial import ConvexHull
from copy import deepcopy

class Robot:
    """
    Robot Class for configuration
    State must be updated when robot move.
    """
    def __init__(self):
        self.name = None
        self.index = None
        self.pos = (0.0, 0.0)
        self.state = [0.0, 0.0, 90, 0, 0]
        self.robot_radius = 0.5
        self.service = []
        self.action_cost = []
        self.path = None
        self.path_time = 0
        # Robot For Local Planning
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 2.0
        self.speed_cost_gain = 10.0
        self.obstacle_cost_gain = 1.0

    def robot_pos(self):
        return self.pos

class Environment:
    def __init__(self, yaml_file = None):
        self.yaml_file = yaml_file
        self.environment_loaded = False
        self.bounds = []
        self.init_num_of_samples = 0
        self.num_of_robot = 0
        self.num_of_goal = 0
        self.obstacles = []
        self.obstacles_map = {}
        self.robots = []
        self.robots_map = {}
        self.goals = []
        self.goals_map = {}

        self.obstacle_area = 0
        self.map_area = 0
        self.free_area = 0

        if not yaml_file is None:
            if self.load_from_yaml_file(yaml_file):
                if not self.bounds:
                    self.calculate_scene_dimensions()
                self.environment_loaded = True
    @property
    def bounds(self):
        return self.__bounds
    @bounds.setter
    def bounds(self, bound_list):
        self.__bounds = bound_list

    def add_obstacles(self, obstacles):
        self.obstacles = self.obstacles + obstacles
        self.calculate_scene_dimensions()

    # Cacluate Map Bounds
    def calculate_scene_dimensions(self):
        points = []
        for elem in self.obstacles:
            points = points + list(elem.boundary.coords)
        mp = geom.MultiPoint(points)
        self.bounds = mp.bounds

    def load_from_yaml_file(self, yaml_file):
        f = Path(yaml_file)
        if not f.is_file():
            raise BaseException(yaml_file + " does not exist.")
        f = open(yaml_file, 'r')
        self.data = yaml.safe_load(f)
        f.close()
        # Randomized Map Generation
        if 'random' in self.data:
            print("Generate Random Goals and Random Obstacles")
            return self.make_random_data(self.data)
        # Fixed Map Generation
        else:
            return self.parse_yaml_data(self.data)

    def parse_yaml_data(self, data):
        data_loaded = False
        if 'robots' in data:
            robots = data['robots']
            self.num_of_robot = robots['num_of_robot']
            self.parse_yaml_robots(robots['robot'])
            if self.num_of_robot != len(self.robots):
                raise Exception("Number of robots is different.")
            data_loaded = True
        if 'environment' in data:
            env = data['environment']
            self.parse_yaml_obstacles(env['obstacles'])
            bounds = env['bounds'].split(', ')
            self.init_num_of_samples = env['num_of_samples']
            self.bounds = [int(bound) for bound in bounds]
            self.bounds = tuple(self.bounds)
            minx, miny, maxx, maxy = self.bounds
            self.map_area = (maxx-minx) * (maxy - miny)
            data_loaded = True
        if 'goals' in data:
            goals = data['goals']
            self.num_of_goal = goals['num_of_goals']
            self.parse_yaml_goals(goals['goal'])
            if self.num_of_goal != len(self.goals):
                raise Exception("Number of goals is dfferent.")
            data_loaded = True
        self.free_area = self.map_area - self.obstacle_area
        return data_loaded

    def make_random_data(self, data):
        """
        Make Randomized Map Instance
        Build Obstacles - Build Robot - Build Goals
        """
        data_loaded = False

        robot_task = []
        task_type = []
        buffer_points = []
        margin_bounds = []

        # Random Parameter Setting
        random_ins = data['random']
        bounds = random_ins['bounds'].split(', ')
        self.init_num_of_samples = random_ins['number_of_samples']
        self.bounds = [int(bound) for bound in bounds]
        self.bounds = tuple(self.bounds)
        minx, miny, maxx, maxy = self.bounds
        self.map_area = (maxx-minx) * (maxy - miny)
        max_goals = random_ins['max_num_goals']
        if 'max_num_obstacles' in random_ins:
            max_obs = random_ins['max_num_obstacles']
        else:
            max_obs = 0
        diameter = random_ins['diameter']

        margin_bounds = (minx + diameter, miny + diameter, maxx - diameter, maxy - diameter)

        # Obstacle Setting
        # Fixed Obstacles
        if 'obstacles' in data:
            self.parse_yaml_obstacles(data['obstacles'])
        # Random Obstacles
        else:
            obstacle_num = random.randint(0, max_obs)
            self.obstacles = []
            self.obstacles_map = {}
            try_count = 0
            obs_count = 0
            obs_name = 'obstacle'
            while obs_count < obstacle_num:
                if try_count > 10000:
                    break
                vertice_num = random.randint(4, 7)
                obstacle = self.simple_polygon(vertice_num)
                overlap = False
                for ele in buffer_points:
                    if obstacle.intersects(ele):
                        overlap = True
                        break
                if not overlap:
                    name = obs_name + str(obs_count)
                    obstacle.name = name
                    self.obstacles.append(obstacle)
                    self.obstacles_map[name] = obstacle
                    self.obstacle_area += obstacle.area
                    buffer_points.append(obstacle)
                    obs_count += 1
                try_count += 1

        # Robot Setting
        if 'robots' in data:
            robots = data['robots']
            self.num_of_robot = robots['num_of_robot']
            self.robots = []
            self.robots_map = {}
            for i, (name, description) in enumerate(robots['robot'].items()):
                parsed = Robot()
                parsed.name = name
                r1 = None
                # sample random point
                while True:
                    overlap = False
                    r1 = self.get_random_point(margin_bounds)
                    buffered_point = geom.Point(r1).buffer(diameter*1.25)
                    for ele in buffer_points:
                        if buffered_point.intersects(ele):
                            overlap = True
                            break
                    if not overlap:
                        buffer_points.append(buffered_point)
                        break
                parsed.pos = r1
                parsed.state[0] = r1[0]
                parsed.state[1] = r1[1]
                parsed.index = i
                if 'robot_radius' in description:
                    parsed.robot_radius = description['robot_radius']
                if 'service' in description:
                    parsed.service = description['service'].split(', ')
                    for task in parsed.service:
                        if not task in robot_task:
                            robot_task.append(task)
                            task_type.append('single_task')
                if 'action_cost' in description:
                    parsed.action_cost = description['action_cost']
                buffer_points.append(geom.Point(parsed.pos).buffer(parsed.robot_radius*1.25))
                self.robots.append(parsed)
                self.robots_map[name] = parsed
        else:
            raise Exception("Robot instance not specified")

        # Goal Setting
        goal_num = max_goals
        # goal_num = random.randint(self.num_of_robot, max_goals)
        self.goals = []
        self.goals_map = {}
        goal_center = []
        for i in range(goal_num):
            goal_name = 'goal'+str(i)
            g1 = None
            parsed = None
            # sample random point
            while True:
                overlap = False
                g1 = self.get_random_point(margin_bounds)
                goal_points = []
                for point in [(-0.35, -0.35), (-0.35, 0.35), (0.35, -0.35), (0.35, 0.35)]:
                    new_point = [round(point[0]+g1[0],2), round(point[1]+g1[1],2)]
                    goal_points.append(new_point)
                for points in itertools.permutations(goal_points):
                    polygon = geom.Polygon(points)
                    polygon.name = goal_name
                    if polygon.is_valid:
                        parsed = polygon
                        break
                for ele in buffer_points:
                    if parsed.intersects(ele):
                        overlap = True
                        break
                for ele in goal_center:
                    if get_distance(g1, ele) < 3.0:
                        overlap = True
                        break
                if not overlap:
                    goal_center.append(g1)
                    buffer_points.append(parsed)
                    break

            self.goals.append(parsed)
            self.goals_map[goal_name] = dict()
            self.goals_map[goal_name]['based'] = 'region'
            self.goals_map[goal_name]['service'] = robot_task
            self.goals_map[goal_name]['service_type'] = task_type
            self.goals_map[goal_name]['shape'] = parsed
            self.goals_map[goal_name]['index'] = i
            self.goals_map[goal_name]['order'] = []
            ### Need to order Constraint

        self.free_area = self.map_area - self.obstacle_area
        data_loaded = True
        return data_loaded

    def parse_yaml_robots(self, robots):
        self.robots = []
        self.robots_map = {}
        for i, (name, description) in enumerate(robots.items()):
            if name.find("__") != -1:
                raise Exception("Names cannot contain double underscores.")
            parsed = Robot()
            parsed.name = name
            if 'state' in description:
                state = description['state']
                parsed.pos = (state[0], state[1])
                parsed.state = state
                parsed.state[2] = state[2] * math.pi / 180.0
                parsed.state[4] = state[4] * math.pi / 180.0
                parsed.index = i
            if 'robot_radius' in description:
                parsed.robot_radius = description['robot_radius']
            if 'ctr_parameter' in description:
                parameter = description['ctr_parameter']
                parsed.max_speed = parameter[0]
                parsed.min_speed = parameter[1]
                parsed.max_yaw_rate = parameter[2] * math.pi / 180.0
                parsed.max_accel = parameter[3]
                parsed.max_yaw_rate = parameter[4] * math.pi / 180.0
            if 'dwa_parameter' in description:
                parameter = description['dwa_parameter']
                parsed.dt = parameter[0]
                parsed.predict_time = parameter[1]
                parsed.to_goal_cost_gain = parameter[2]
                parsed.speed_cost_gain = parameter[3]
                parsed.obstacle_cost_gain = parameter[4]
            if 'service' in description:
                parsed.service = description['service'].split(', ')
            if 'action_cost' in description:
                parsed.action_cost = description['action_cost']
            self.robots.append(parsed)
            self.robots_map[name] = parsed

    def parse_yaml_goals(self, goals):
        self.goals = []
        self.goals_map = {}
        for i, (name, description) in enumerate(goals.items()):
            if name.find("__") != -1:
                raise Exception("Names cannot contain double underscores.")
            if description['shape'] == 'polygon':
                parsed = self.parse_polygon(name, description)
            else:
                raise Exception("not a rectangle")
            if not parsed.is_valid:
                raise Exception("%s is not valid"%name)
            self.goals.append(parsed)
            self.goals_map[name] = dict()
            self.goals_map[name]['based'] = description['based']
            self.goals_map[name]['service'] = description['service'].split(', ')
            self.goals_map[name]['service_type'] = description['service_type'].split(', ')
            self.goals_map[name]['shape'] = parsed
            self.goals_map[name]['index'] = i
            if description.get('order'):
                for ele in description['order']:
                    self.goals_map[name]['order'] = self.goals_map[name].get('order', []) + [ele.split('-')]
            else:
                self.goals_map[name]['order'] = []
            if description['based'] == 'obstacle':
                self.obstacles.append(parsed)
                self.obstacles_map[name] = parsed
                self.obstacle_area += parsed.area

    def parse_yaml_obstacles(self, obstacles):
        self.obstacles = []
        self.obstacles_map = {}
        if obstacles:
            for name, description in obstacles.items():
                if name.find("__") != -1:
                    raise Exception("Names cannot contain double underscores.")
                if description['shape'] == 'rectangle':
                    parsed = self.parse_rectangle(name, description)
                elif description['shape'] == 'polygon':
                    parsed = self.parse_polygon(name, description)
                else:
                    raise Exception("not a rectangle")
                if not parsed.is_valid:
                    raise Exception("%s is not valid!"%name)
                self.obstacles.append(parsed)
                self.obstacles_map[name] = parsed
                self.obstacle_area += parsed.area
            self.expanded_obstacles = [obs.buffer(0.75/2, resolution = 2) for obs in self.obstacles]

    def parse_rectangle(self, name, description):
        center = description['center']
        center = geom.Point((center[0], center[1]))
        length = description['length']
        width = description['width']
        # convert rotation to radians
        rotation = description['rotation']# * math.pi /180
        corners = [(center.x - length/2., center.y - width/2.),
                    (center.x + length/2., center.y - width/2.),
                    (center.x + length/2., center.y + width/2.),
                    (center.x - length/2., center.y + width/2.)]
        # print corners
        polygon = geom.Polygon(corners)
        out = affinity.rotate(polygon, rotation, origin=center)
        out.name = name
        out.cc_length = length
        out.cc_width = width
        out.cc_rotation = rotation
        return out

    def parse_polygon(self, name, description):
        _points = description['corners']
        for points in itertools.permutations(_points):
            polygon = geom.Polygon(points)
            polygon.name = name
            if polygon.is_valid:
                return polygon

    def save_to_yaml(self, yaml_file):
        yaml_dict = {}
        obstacles = {}
        for i, ob in enumerate(self.obstacles):
            ob_dict = {}
            ob_dict['shape'] = 'polygon'
            ob_dict['corners'] = [list(t) for t in list(ob.boundary.coords)]
            ob_name = "obstacle%.4d"%i
            obstacles[ob_name] = ob_dict
        yaml_dict['environment'] = {'obstacles' : obstacles}

        f = open(yaml_file, 'w')
        f.write(yaml.dump(yaml_dict, default_flow_style=None))
        f.close()

    def get_random_point(self, bounds):
        """
        Get Random Point
        @Return (x,y) inside of boundary
        """
        x = bounds[0] + random.random()*(bounds[2]-bounds[0])
        x = round(x, 2)
        y = bounds[1] + random.random()*(bounds[3]-bounds[1])
        y = round(y, 2)
        return (x,y)

    def simple_polygon(self, numVert):
        minx, miny, maxx, maxy = self.bounds
        # Create an array of random points
        points = []
        buffer_size = random.randint(10,40) / 10
        x_s = []
        y_s = []
        for i in range(0, int(numVert)):
            x = round(random.random()*buffer_size,2)
            y = round(random.random()*buffer_size,2)
            points.append([x, y])
            x_s.append(x)
            y_s.append(y)
        randx = random.randint(int((minx-0.5-min(x_s))*100), int((maxx-0.5-max(x_s))*100)) / 100
        randy = random.randint(int((miny-0.5-min(y_s))*100), int((maxy-0.5-max(y_s))*100)) / 100
        for ele in points:
            ele[0] += randx
            ele[1] += randy

        points = np.array(points)
        hull = ConvexHull(points)
        # Order the edges of the convex hull
        simplices = []
        simplices = deepcopy(hull.simplices)
        for i in range(1, len(hull.simplices)):
            edge_ant = simplices[i-1]
            for j in range(0, len(hull.simplices)):
                edge = hull.simplices[j]
                if ( (edge_ant[1] == edge[0]) and (edge_ant[0] != edge[1]) ):
                    simplices[i] = [edge_ant[1], edge[1]]
                    break
                elif( (edge_ant[1] == edge[1]) and (edge_ant[0] != edge[0]) ):
                    simplices[i] = [edge_ant[1], edge[0]]
                    break

        hull.simplices = simplices

        # Look for points that are not part of the convex hull
        idx = []
        for i in range(0, len(points)):
            pt = [points[i, 0], points[i, 1]]
            equal = []
            for vert in hull.vertices:
                equal.append( (pt[0] == hull.points[vert,0]) and (pt[1] == hull.points[vert,1]) )

            if sum(equal) == 0:
                dist_to_edge = []
                for edge in hull.simplices:
                    # distance to each edge of the convex hull
                    A = hull.points[edge[0]]
                    B = hull.points[edge[1]]
                    vec1 = [A[0] - B[0], A[1] - B[1]]
                    vec2 = [A[0] - pt[0], A[1] - pt[1]]
                    dist_to_edge.append(length([np.cross(vec1, vec2),0])/length(vec1))
                # save the index of the point to introduce and the edge to break
                idx.append([i, dist_to_edge.index(min(dist_to_edge))])

        poligono = []
        # Create a new polygon which includes all the points
        if len(idx) == 0:
            for vert in hull.vertices:
                poligono.append(hull.points[vert])

        else:
            #poligono.append(hull.points[hull.simplices[0,0]])
            for i in range(0, len(hull.simplices)):
                break_this_edge = []
                dist = []

                for change in idx:
                    break_this_edge.append(i == change[1])
                    if (break_this_edge[-1]):
                        dist.append(length(hull.points[hull.simplices[i,0]]-points[change[0]]))
                    else:
                        # this number is higher than the maximum possible distance
                        dist.append(10)
                poligono.append(hull.points[hull.simplices[i,0]])
                if sum(break_this_edge) == 1:
                    j = break_this_edge.index(True)
                    pt_idx =  idx[j]

                    poligono.append(points[pt_idx[0]])
                elif sum(break_this_edge) > 1:
                    for k in range(0,sum(break_this_edge)):
                        idx_unir = idx[dist.index(min(dist))]
                        poligono.append(points[idx_unir[0]])
                        dist[dist.index(min(dist))] = 10
        obstacle_points = []
        obstacle = None

        for pt in poligono:
            obstacle_points.append([pt[0], pt[1]])
        obstacle_points.append(obstacle_points[0])

        for points in itertools.permutations(obstacle_points):
            polygon = geom.Polygon(points)
            if polygon.is_valid:
                obstacle = polygon
                break

        return obstacle

