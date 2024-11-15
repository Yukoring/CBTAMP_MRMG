import random
import math
import copy
from utils import *
from shapely.geometry import Point, LineString

class PRMPlanning():
    def __init__(self, environment, vor_sample, strategy):
        """
        Ininital Probabilistic RoadMap
        Env: Enviroment instance from yaml file
        Bounds: Boundary of the map
        Robot: Robot configurations
        isLazy: Lazy Algorithm will be futher added.
        Samples: Dict (robot_radius - 1-D array samples)
        Roadmap: Dict (robot_radius - 2-D array Roadmap)
        skdtree: KDTree for finding nearest samples (Dict)
        """
        self.env = environment
        self.bounds = environment.bounds
        self.robots = environment.robots
        self.robots_map = environment.robots_map
        self.goals_map = environment.goals_map
        self.num_of_robot = environment.num_of_robot
        self.vor_samples = vor_sample
        self.vor_ways = []
        self.strategy = strategy

        # isLazy for Lazy PRM function for future work.
        self.samples = {}
        self.skdtree = {}
        self.roadmaps = {}
        self.sampling_dist = {}
        self.casting_dist = {}

        self.init_num_of_samples = environment.init_num_of_samples
        if self.init_num_of_samples == 0:
            self.init_num_of_samples = 500

        self.max_rr = 0
        for name, robot in self.robots_map.items():
            if self.max_rr < robot.robot_radius:
                self.max_rr = robot.robot_radius
        self.max_samples = []
        self.max_roadmap = []
        self.goal_samples = []
        self.knn_const = 40 # Parameter neighbor for knn (samples / const)

    def dijkstra_planning(self, start, goal, road_map, sample_points):
        """
        start : start position (x,y)
        goal : goal position (x,y)
        road_map : roadmap by generate_roadmap (2d array)
        sample_points : sampled location by sample_waypoint (array)
        @return: lists of path coordinates ([(x1,y1), (x2,y2) ...]), empty list when no path was found
        """
        open_list = dict()
        closed_list = dict()
        start_index = sample_points.index(start)

        root = {'id': start_index, 'loc': start, 'cost': 0.0, 'parent': None}
        open_list[(root['id'])] = root
        closed_list[(root['id'])] = root

        while len(open_list) > 0:
            curr_id = min(open_list, key=lambda o: open_list[o]['cost'])
            curr = open_list[curr_id]
            del open_list[curr_id]
            # Goal Checking
            if curr['loc'] == goal:
                return get_path(curr)
            # search in road_map
            for i in range(len(road_map[curr_id])):
                child_id = road_map[curr_id][i]
                child_loc = sample_points[child_id]
                child_x, child_y = child_loc
                dx = child_x - curr['loc'][0]
                dy = child_y - curr['loc'][1]
                d = math.sqrt(dx**2 + dy**2)
                child = {'id': child_id, 'loc': sample_points[child_id],
                        'cost': curr['cost'] + d, 'parent': curr}
                if (child['id']) in closed_list:
                    existing_node = closed_list[(child['id'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['id'])] = child
                        open_list[(child['id'])] = child
                else:
                    closed_list[(child['id'])] = child
                    open_list[(child['id'])] = child
        return None # Failed to find solution

    def path_smoothing(self, path, rr):
        max_iter = 100

        for i in range(max_iter):
            path_len = len(path)
            # Sample two points
            pickPoints = [random.randint(0,path_len-1), random.randint(0,path_len-1)]
            pickPoints.sort()

            if pickPoints[0] == pickPoints[1]:
                continue
            if pickPoints[1] - pickPoints[0] == 1:
                continue
            first = path[pickPoints[0]]
            second = path[pickPoints[1]]

            # collision check
            if not self.isEdgeCollisionFree(first, second, rr):
                continue
            # Create New path
            newPath = []
            newPath.extend(path[:pickPoints[0] + 1])
            newPath.extend(path[pickPoints[1]:])
            path = newPath
            path_cost = get_sum_of_cost(path)

        return path

    def ConstructPhase(self):
        """
        Construct Roadmap Function
        Goal Related Sampling - Region Based Goal and Obstacle Based Goal

        Sampling - Making KDTree with samples - Constructing Roadmap
        Every different Model uses different Roadmap (Check it with there is key in dict)
        Sampling Distance: Minimum distance between samples
        Casting Distance: Maximum distance connecting the samples

        @Return initial samples and roadmap
        """

        # Setting number of samples
        sample_n = min(max(len(self.env.obstacles)*2, self.init_num_of_samples), 2000)
        parking = []

        # PRM Structure
        for name, robot in self.robots_map.items():
            rr = robot.robot_radius
            if not rr in self.roadmaps:
                # For Sampling Generalization
                # if self.solver == 'Taskonly':
                #     sample_n = self.env.free_area / (rr*rr*math.pi)
                #     sample_n = int(sample_n/3.0)
                #     sd = rr*2 + 0.125
                #     cd = sd * 1.9
                sd = round((self.env.free_area / (rr*rr*math.pi))/ sample_n, 2)
                sd = max(0.5, sd)
                sd = min(2.0, sd)
                sd = sd * rr
                cd = sd * 6
                cd = max(0.75, cd)
                cd = min(2.25, cd)
                self.sampling_dist[rr] = sd
                self.casting_dist[rr] = cd
                self.samples[rr] = self.sample_waypoint(rr, sample_n, self.sampling_dist[rr])
                self.skdtree[rr] = KDTree(self.samples[rr])
                self.roadmaps[rr] = self.generate_roadmap(rr, self.casting_dist[rr])

                if self.strategy == 'goal':
                    self.goal_samples, parking = self.goal_directed_samples(self.samples[rr])

                else:
                    for name2, goal in self.goals_map.items():
                        goal['samples'] = self.simple_goal_samples(goal, self.samples[rr])
                        self.goal_samples.append(goal['samples'])

            if not self.isRoadmapEnough(robot.pos, self.goal_samples, rr):
                raise Exception("Goal cannot be reached!")
        # Setting PRM only for Task Domain
        self.construct_task_elements(self.goal_samples)

        return self.samples, self.roadmaps, self.goal_samples, parking

    def QuerywithRoadmap(self, start_pos, goal_pos, samples, roadmap, skdtree, rr):
        if not start_pos in samples or not goal_pos in samples:
            query_samples, query_roadmap = self.add_single_query(start_pos, goal_pos, samples, roadmap, rr, skdtree=skdtree)
            if start_pos == goal_pos:
                path1 = [start_pos, goal_pos]
                path2 = [goal_pos, start_pos]
            elif get_distance(start_pos, goal_pos) < rr * 5 and self.isEdgeCollisionFree(start_pos, goal_pos, rr):
                task_path1, task_cost1 = self.QueryTaskPlan(start_pos,goal_pos)
                task_path2, task_cost2 = self.QueryTaskPlan(goal_pos,start_pos)
                if task_cost1 != -1:
                    path1 = task_path1
                else:
                    path1 = [start_pos, goal_pos]
                if task_cost2 != -1:
                    path2 = task_path2
                else:
                    path2 = [goal_pos, start_pos]
            else:
                path1 = self.dijkstra_planning(start_pos, goal_pos, query_roadmap, query_samples)
                # if path1:
                #     path1 = self.path_smoothing(path1, rr)
                path2 = self.dijkstra_planning(goal_pos, start_pos, query_roadmap, query_samples)
                # if path2:
                #     path2 = self.path_smoothing(path2, rr)
        else:
            if start_pos == goal_pos:
                path1 = [start_pos, goal_pos]
                path2 = [goal_pos, start_pos]
            elif get_distance(start_pos, goal_pos) < rr * 5 and self.isEdgeCollisionFree(start_pos, goal_pos, rr):
                task_path1, task_cost1 = self.QueryTaskPlan(start_pos,goal_pos)
                task_path2, task_cost2 = self.QueryTaskPlan(goal_pos,start_pos)
                if task_cost1 != -1:
                    path1 = task_path1
                else:
                    path1 = [start_pos, goal_pos]
                if task_cost2 != -1:
                    path2 = task_path2
                else:
                    path2 = [goal_pos, start_pos]
            else:
                path1 = self.dijkstra_planning(start_pos, goal_pos, roadmap, samples)
                # if path1:
                #     path1 = self.path_smoothing(path1, rr)
                path2 = self.dijkstra_planning(goal_pos, start_pos, roadmap, samples)
                # if path2:
                #     path2 = self.path_smoothing(path2, rr)

        if path1 is None or path2 is None:
            cost1 = -1
            cost2 = -1
        else:
            cost1 = round(get_sum_of_cost(path1), 2)
            cost2 = round(get_sum_of_cost(path2), 2)
            if cost1 < cost2:
                path2 = copy.deepcopy(path1)
                path2.reverse()
                cost2 = round(get_sum_of_cost(path2), 2)
            else:
                path1 = copy.deepcopy(path2)
                path1.reverse()
                cost1 = round(get_sum_of_cost(path1), 2)

        return path1, cost1, path2, cost2

    def QuerySingle(self, start_pos, goal_pos, rr):
        path = None

        if start_pos == goal_pos:
            path = [start_pos, goal_pos]
        elif self.isEdgeCollisionFree(start_pos, goal_pos, rr) and \
            get_distance(start_pos, goal_pos) < rr*5:
            #print("No Edge Collision occurs")
            path = [start_pos, goal_pos]
        else:
            start = [start_pos]
            goals = [goal_pos]
            queried_samples, queried_roadmap = self.add_query(start, goals, rr)
            path = self.dijkstra_planning(start_pos, goal_pos, queried_roadmap, queried_samples)
            # if path:
            #     path = self.path_smoothing(path, rr)
        if path:
            cost = get_sum_of_cost(path)
            return path, round(cost,2)
        else:
            return None, -1

    def QueryTaskPlan(self, start_pos, goal_pos):
        #print(len(self.max_samples), len(self.max_roadmap))
        path = None
        if start_pos == goal_pos:
            path = [start_pos, goal_pos]
        elif self.isEdgeCollisionFree(start_pos, goal_pos, self.max_rr) and \
            get_distance(start_pos, goal_pos) < self.max_rr*3:
            #print("No Edge Collision occurs")
            path = [start_pos, goal_pos]
        else:
            path = self.dijkstra_planning(start_pos, goal_pos, self.max_roadmap, self.max_samples)
            # if path:
            #     path = self.path_smoothing(path, self.max_rr)
        if path:
            cost = get_sum_of_cost(path)
            return path, round(cost,2)
        else:
            return None, -1

    def isRoadmapEnough(self, start_pos, goal_samples, rr):
        """
        Check reach every goal sample from start position
        start_pos = one robot start position
        goal_samples = every goal_samples
        rr = Robot Radius
        @Return True if every goal samples are reached from position, else reconstruction or False
        """
        start = [start_pos]
        goals = []
        for ele in goal_samples:
            goals += ele

        queried_samples, queried_roadmap = self.add_query(start, goals, rr)
        for goal in goals:
            path = None
            if start_pos == goal:
                path = [start_pos, goal]
            while path is None:
                if len(self.samples[rr]) > 1500:
                    print("Already too many samples")
                    return False
                path = self.dijkstra_planning(start_pos, goal, queried_roadmap, queried_samples)
                if path is None:
                    print("Roadmap Reconstruction")
                    sd = self.sampling_dist[rr]
                    cd = self.casting_dist[rr]
                    self.samples[rr], self.roadmaps[rr] = self.roadmap_reconstruct(self.samples[rr], rr, sd, cd)
                    queried_samples, queried_roadmap = self.add_query(start, goals, rr)
        return True

    def roadmap_reconstruct(self, samples, rr, sd, cd):
        """
        Roadmap Reconstruct Function
        samples: Existed Sample
        rr: Robot Radius
        sd: Sampling Distance
        cd: Casting Distance
        @Return 1-D array 100 num added sample, 2-D array corresponding Roadmap
        """
        if self.solver == 'Taskonly':
            sample_points = []
            nsample = len(samples)
        else:
            sample_points = copy.deepcopy(samples)
            nsample = len(samples) + 100
        # ReSampling with only uniform sampling method
        count = 0
        while len(sample_points) < nsample:
            if count > 20000:
                break
            casting_flag = False
            q = self.find_random_collision_free_configuration(rr)
            for sample in sample_points:
                if get_distance(sample, q) < sd:
                    casting_flag = True
                    break
            if casting_flag == False:
                sample_points.append(q)
            count += 1
        nsample = len(sample_points)
        # KDTree for NN
        knn = min(nsample/self.knn_const, 15)
        knn = max(knn, 5)
        re_roadmap = []
        self.skdtree[rr] = KDTree(sample_points)
        skdtree = self.skdtree[rr]
        # Making Roadmap
        for (i, temp_point) in zip(range(nsample), sample_points):
            index, dists = skdtree.search(temp_point, k=nsample)
            edge_id = []
            # 0 is self so start with 1
            for j in range(1, len(index)):
                npoint = sample_points[index[j]]
                if self.isEdgeCollisionFree(temp_point, npoint, rr) and get_distance(temp_point, npoint) < cd:
                    edge_id.append(index[j])
                if len(edge_id) >= knn:
                    break
            re_roadmap.append(edge_id)

        return sample_points, re_roadmap

    def add_single_query(self, start, goal, samples, roadmap, rr, skdtree = None):
        query_samples = copy.deepcopy(samples)
        query_roadmap = copy.deepcopy(roadmap)
        knn = min(len(self.samples[rr])/self.knn_const, 15)
        knn = max(knn, 5)
        sample_len = len(samples)
        cd = self.casting_dist[rr]

        if skdtree is None:
            skdtree = self.skdtree[rr]

        if not start in query_samples:
            query_samples.append(start)
            index, dists = skdtree.search(start, k = knn)
            edge_id = []
            for i in range(0, len(index)):
                npoint = samples[index[i]]
                if self.isEdgeCollisionFree(start, npoint, rr) and get_distance(start, npoint) < cd:
                    edge_id.append(index[i])
                if len(edge_id) >= knn:
                    break
            query_roadmap.append(edge_id)

            for ele in edge_id:
                query_roadmap[ele].append(len(query_samples)-1)

        if not goal in query_samples:
            query_samples.append(goal)
            index, dists = skdtree.search(goal, k = knn)
            edge_id = []
            for i in range(0, len(index)):
                npoint = samples[index[i]]
                if self.isEdgeCollisionFree(goal, npoint, rr) and get_distance(goal, npoint) < cd:
                    edge_id.append(index[i])
                if len(edge_id) >= knn:
                    break
            query_roadmap.append(edge_id)

            for ele in edge_id:
                query_roadmap[ele].append(len(query_samples)-1)

        return query_samples, query_roadmap

    def add_query(self, starts, goals, rr):
        """
        Adding start and goal position to given sample and roadmap
        Add Query means they are using existed sample and roadmap.
        start: list of start positions of robots
        goal: list of goal positions
        samples: 1-D array Samples
        rr: Robot Radius
        @Return 1-D array queried samples, 2-D array queried roadmap
        """
        samples = self.samples[rr]
        roadmap = self.roadmaps[rr]
        query_samples = copy.deepcopy(samples)
        query_roadmap = copy.deepcopy(roadmap)
        skdtree = self.skdtree[rr]
        cd = self.casting_dist[rr]

        # Add start pos and goal pos to sample
        add_sample = []
        temp_samples = starts+goals
        for point in temp_samples:
            if not point in query_samples:
                add_sample.append(point)
        query_samples += add_sample
        nsample = len(query_samples)

        knn = min(int(nsample/self.knn_const), 15)
        knn = max(knn, 5)

        sample_len = len(samples)
        for sample in add_sample:
            index, dists = skdtree.search(sample, k = knn)
            edge_id = []
            for i in range(0, len(index)):
                npoint = samples[index[i]]
                if self.isEdgeCollisionFree(sample, npoint, rr) and get_distance(sample, npoint) < cd:
                    edge_id.append(index[i])
                if len(edge_id) >= knn:
                    break
            query_roadmap.append(edge_id)
            for ele in edge_id:
                query_roadmap[ele].append(sample_len)
            sample_len += 1

        return query_samples, query_roadmap

    def sample_waypoint(self, rr, sample_n, sd):
        """
        Sampling Waypoint Function
        rr: Robot radius
        sample_n: Number of Samples
        sd: Minimum Sampling Distance between samples
        @Return 1-D array Samples
        """

        # Initial setting for vornoi samples, robot initial position, goal positin if they are region-based
        sample_points = copy.deepcopy(self.vor_samples)
        for i, j in self.robots_map.items():
            sample_points.append(j.pos)
        for i, j in self.goals_map.items():
            if j['based'] == 'region':
                polygon = j['shape']
                sample_points.append(polygon.centroid.coords[0])

        # There are tons of method for sampling such as bridge, gaussian, ...
        # We use bridge sampling for 1/3 samples and uniform sampling for the others.

        count = 0
        while len(sample_points) < sample_n/3:
            if count > 20000:
                break
            casting_flag = False
            q = self.find_bridge_collision_free_configuration(rr)
            for sample in sample_points:
                if get_distance(sample, q) < sd:
                    casting_flag = True
                    break
            if casting_flag == False:
                sample_points.append(q)
            count += 1
        # Uniform Sample (2/3)
        count = 0
        while len(sample_points) < sample_n:
            if count > 20000:
                break
            casting_flag = False
            q = self.find_random_collision_free_configuration(rr)
            for sample in sample_points:
                if get_distance(sample, q) < sd:
                    casting_flag = True
                    break
            if casting_flag == False:
                sample_points.append(q)
            count += 1

        random.shuffle(sample_points)

        return sample_points

    def goal_directed_samples(self, cand_samples):
        """
        Goal-directed region sampling
        polygon input
        @return goal samples inside region
        """
        task_number = dict()
        goal_samples = dict()
        just_samples = []
        output_goal = []
        task_config = dict()
        temp_list = []
        parking = []


        for name, goal in self.goals_map.items():
            task_number[name] = len(goal['service'])
            polygon = goal['shape']
            if goal['based'] == 'obstacle':
                access_dist = self.max_rr*2.0
                minx = min(polygon.exterior.coords.xy[0]) - access_dist
                maxx = max(polygon.exterior.coords.xy[0]) + access_dist
                miny = min(polygon.exterior.coords.xy[1]) - access_dist
                maxy = max(polygon.exterior.coords.xy[1]) + access_dist
            else:
                minx = min(polygon.exterior.coords.xy[0])
                maxx = max(polygon.exterior.coords.xy[0])
                miny = min(polygon.exterior.coords.xy[1])
                maxy = max(polygon.exterior.coords.xy[1])

            for sample in cand_samples:
                if sample[0] > minx and sample[0] < maxx and sample[1] > miny and sample[1] < maxy:
                    task_config[sample] = task_config.get(sample, []) + [name]
                    temp_list.append(sample)

        while len(task_config) > 0:
            task_config = dict(sorted(task_config.items(), key= lambda x: len(x[1])))
            cand = task_config.popitem()
            cand_pos = cand[0]
            cand_goal = cand[1]

            overlap_flag = False
            for sample in just_samples:
                if get_distance(sample, cand_pos) < self.max_rr*2:
                    overlap_flag = True
                    break
            if not overlap_flag:
                for goal in cand_goal:
                    if task_number[goal] > 0:
                        goal_samples[goal] = goal_samples.get(goal, []) + [cand_pos]
                        task_number[goal] -= 1
                        if not cand_pos in just_samples:
                            just_samples.append(cand_pos)
                delete_list = []
                for i, j in task_config.items():
                    if get_distance(cand_pos, i) < self.max_rr*2.1:
                        delete_list.append(i)
                for i in delete_list:
                    task_config.pop(i)

        for name, number in task_number.items():
            extend = 0
            polygon = self.goals_map[name]['shape']
            minx = min(polygon.exterior.coords.xy[0])
            maxx = max(polygon.exterior.coords.xy[0])
            miny = min(polygon.exterior.coords.xy[1])
            maxy = max(polygon.exterior.coords.xy[1])
            while number > 0:
                extend += self.max_rr
                eminx = minx - extend
                emaxx = maxx + extend
                eminy = miny - extend
                emaxy = maxy + extend
                out_candi = []
                for sample in cand_samples:
                    if sample[0] > eminx and sample[0] < emaxx and sample[1] > eminy and sample[1] < emaxy and (not sample in temp_list):
                        out_candi.append(sample)
                for sample in out_candi:
                    if number < 1:
                        break
                    overlap_flag = False
                    for js in just_samples:
                        if get_distance(sample, js) < self.max_rr*2:
                            overlap_flag = True
                            break
                    if not overlap_flag:
                        just_samples.append(sample)
                        # goal_samples[name] = goal_samples.get(name, []) + [sample]
                        parking.append(sample)
                        number -= 1

        for name, goal in self.goals_map.items():
            goal['samples'] = goal_samples[name]
            output_goal.append(goal_samples[name])

        return output_goal, parking




    def region_goal_samples(self, polygon):
        """
        Goal Region Sampling
        polygon input
        @Return goal samples inside region
        """
        sample_points = []
        sample_n = 1+(polygon.area//((self.max_rr*2.0)**2))
        sd = polygon.area / sample_n
        sample_points.append(polygon.centroid.coords[0])
        minx = min(polygon.exterior.coords.xy[0])
        maxx = max(polygon.exterior.coords.xy[0])
        miny = min(polygon.exterior.coords.xy[1])
        maxy = max(polygon.exterior.coords.xy[1])
        bounds = (minx, miny, maxx, maxy)

        count = 0
        while len(sample_points) < sample_n:
            if count > 200 and len(sample_points) > 0:
                break
            elif count > 500:
                raise Exception("Can't Sample in Goal Region")
            casting_flag = False
            q = self.get_random_point(bounds)
            for sample in sample_points:
                if get_distance(sample, q) < sd*3.0:
                    casting_flag = True
                    break
            if not self.isPointCollisionFree(q, self.max_rr) or self.isOutOfBounds(q, self.max_rr):
                casting_flag = True
            if casting_flag == False:
                sample_points.append(q)
            count += 1
        return sample_points

    def simple_goal_samples(self, goal, cand_samples):
        """
        Goal Obstacle Sampling
        polygon input
        @Return goal samples near goal obstacle
        """
        sample_points = []
        polygon = goal['shape']
        # sample_n = 1+ (polygon.area//((self.max_rr*2)**2))
        if goal['based'] == 'obstacle':
            access_dist = self.max_rr*2.0
            minx = min(polygon.exterior.coords.xy[0]) - access_dist
            maxx = max(polygon.exterior.coords.xy[0]) + access_dist
            miny = min(polygon.exterior.coords.xy[1]) - access_dist
            maxy = max(polygon.exterior.coords.xy[1]) + access_dist
        else:
            minx = min(polygon.exterior.coords.xy[0])
            maxx = max(polygon.exterior.coords.xy[0])
            miny = min(polygon.exterior.coords.xy[1])
            maxy = max(polygon.exterior.coords.xy[1])

        goal_sample = []
        for goal in self.goal_samples:
            goal_sample += goal

        candidate = []
        for sample in cand_samples:
            if minx < sample[0] and maxx > sample[0] and miny < sample[1] and maxy > sample[1]:
                candidate.append(sample)

        for sample in candidate:
            candi_flag = True
            for gs in goal_sample:
                if get_distance(sample, gs) < self.max_rr*2:
                    candi_flag = False
                    break
            if candi_flag:
                sample_points.append(sample)
                break
        return sample_points


    def generate_roadmap(self, rr, cd):
        """
        RoadMap Geneation Function
        rr: Robot Radius
        cd: Casting Distance
        @Return 2-D array Roadmap
        """
        roadmap = []
        samples = self.samples[rr]
        skdtree = self.skdtree[rr]

        nsample = len(samples)
        knn = min(int(nsample/self.knn_const), 15)
        knn = max(knn, 5)

        for (i, temp_point) in zip(range(nsample), samples):
            index, dists = skdtree.search(temp_point, k=nsample)
            edge_id = []
            # 0 is self so start with 1
            for j in range(1, len(index)):
                npoint = samples[index[j]]
                if self.isEdgeCollisionFree(temp_point, npoint, rr) and get_distance(temp_point, npoint) < cd:
                    edge_id.append(index[j])
                if len(edge_id) >= knn:
                    break
            roadmap.append(edge_id)
        return roadmap

    def construct_task_elements(self, goal_samples):
        """
        Make task planning elements
        goal_samples: goal samples list
        @Update max_samples, max_roadmap
        """
        max_rr = self.max_rr
        starts = []
        goals = []
        for name, ele in self.robots_map.items():
            starts.append(ele.pos)
        for ele in goal_samples:
            goals += ele
        self.max_samples, self.max_roadmap = self.add_query(starts, goals, max_rr)

    def find_nearest_sample(self, q, rr):
        """
        Find Nearest Sample Function
        q: Query Configuration
        rr: Robot radius
        @Return nearest configuration in sample skdTree
        """
        samples = self.samples[rr]
        skdtree = self.skdtree[rr]
        index, dists = skdtree.search(q, 1)
        return samples[index]

    def find_random_collision_free_configuration(self, rr):
        """
        Unifrom Sampling Function
        rr: Robot radius
        @Return free configuration with uniform sampling
        """
        while True:
            # get a random point
            q = self.get_random_point(self.bounds)
            if self.isPointCollisionFree(q, rr) and not self.isOutOfBounds(q, rr):
                return q

    def find_bridge_collision_free_configuration(self, rr):
        """
        Bridge Sampling Function
        rr: Robot Radius
        @Return free configuration with Bridge sampling
        Strategy: Find two obstacle configurations and find mid config between them.
        If that config is free config, return it.
        """
        while True:
            # get random two points
            bounds = self.bounds
            x_margin = (bounds[2] - bounds[0]) / 10
            y_margin = (bounds[3] - bounds[1]) / 10
            bounds = (bounds[0]-x_margin, bounds[1] - y_margin, bounds[2]+x_margin, bounds[3]+y_margin)
            # Bound obstacle Margin or not
            q1 = self.get_random_point(self.bounds)
            q2 = self.get_random_point(self.bounds)
            if ((not self.isPointCollisionFree(q1, rr)) or self.isOutOfBounds(q1, rr)) and \
                ((not self.isPointCollisionFree(q2, rr)) or self.isOutOfBounds(q2, rr)):
                q = (round((q1[0]+q2[0])/2, 2), round((q1[1]+q2[1])/2, 2))
                if self.isPointCollisionFree(q, rr) and not self.isOutOfBounds(q, rr):
                    return q

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

    def isPointCollisionFree(self, q, rr):
        """
        Check given configuration is on collision
        q: Configuration
        rr: Robot Radius
        @Return True if there is no collision, else False
        """
        buffered_point = Point(q).buffer(rr)
        for obstacle in self.env.obstacles:
            if obstacle.intersects(buffered_point):
                return False
        return True

    def isEdgeCollisionFree(self, q1, q2, rr):
        """
        Check given two configuration connect or not
        q1, q2: Configuration
        rr: Robot Radius
        @Return True if no obstacle between two points, else False
        """
        line = LineString([q1, q2])
        expanded_line = line.buffer(rr)
        for obstacle in self.env.obstacles:
            if expanded_line.intersects(obstacle):
                return False
        return True

    def isOutOfBounds(self, q, rr):
        """
        Check config is out of boundary
        q: Configuration
        rr: Robot Radius
        @Return True if out of boundary, else False
        """
        bounds = self.bounds
        if((q[0]-rr) < bounds[0]):
            return True
        if((q[1]-rr) < bounds[1]):
            return True
        if((q[0]+rr) > bounds[2]):
            return True
        if((q[1]+rr) > bounds[3]):
            return True
        return False


