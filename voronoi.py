"""
Voronoi Road Map Planner
"""
import numpy as np
from random import shuffle
from scipy.spatial import Voronoi
from utils import get_distance
from shapely.geometry import Point, LineString


class VoronoiPlanning:
    def __init__(self, env):
        self.env = env
        self.bounds = env.bounds
        self.obstacles = env.obstacles
        self.robot_radius = []
        for i in range(env.num_of_robot):
            self.robot_radius.append(env.robots[i].robot_radius)
        self.max_radius = max(self.robot_radius)
        self.vor_samples = []

    def voronoi_sampling(self):
        minx, miny, maxx, maxy = self.bounds

        ox = []
        oy = []

        # Bound as an obstacle
        for v in np.arange(minx, maxx+0.1, 0.1):
            ox.append(v)
            oy.append(miny)
        for v in np.arange(minx, maxx+0.1, 0.1):
            ox.append(v)
            oy.append(maxy)
        for v in np.arange(miny, maxy+0.1, 0.1):
            ox.append(minx)
            oy.append(v)
        for v in np.arange(miny, maxy+0.1, 0.1):
            ox.append(maxx)
            oy.append(v)

        for obs in self.obstacles:
            a = list(obs.boundary.coords)
            for i in range(len(a)-1):
                x1, y1 = a[i]
                x2, y2= a[i+1]
                ox.append(x1)
                oy.append(y1)
                if x1 == x2:
                    if y1 > y2:
                        for v in np.arange(y2, y1, 0.1):
                            ox.append(x1)
                            oy.append(v)
                    elif y2>y1:
                        for v in np.arange(y1, y2, 0.1):
                            ox.append(x1)
                            oy.append(v)
                elif y1==y2:
                    if x1 > x2:
                        for v in np.arange(x2, x1, 0.1):
                            ox.append(v)
                            oy.append(y1)
                    elif x2 > x1:
                        for v in np.arange(x1, x2, 0.1):
                            ox.append(v)
                            oy.append(y1)

        oxy = np.vstack((ox,oy)).T

        vor_sample = Voronoi(oxy)

        sample_x = [ix for [ix, _] in vor_sample.vertices]
        sample_y = [iy for [_, iy] in vor_sample.vertices]
        v_sample = []

        for i in range(len(sample_x)):
            sx = sample_x[i]
            sy = sample_y[i]
            if sx > maxx or sx < minx or sy > maxy or sy <miny:
                break
            p = Point(sx, sy)
            p_buffer = p.buffer(self.max_radius)
            contain_flag = False
            for obs in self.obstacles:
                if obs.intersects(p_buffer):
                    contain_flag = True
                    break
            line = []
            line.append(LineString([(minx,miny), (minx, maxy)]))
            line.append(LineString([(minx,miny), (maxx, miny)]))
            line.append(LineString([(minx,maxy), (maxx, maxy)]))
            line.append(LineString([(maxx,miny), (maxx, maxy)]))

            for l in line:
                if l.distance(p) < self.max_radius:
                    contain_flag = True
                    break

            if not contain_flag:
                v_sample.append((sx, sy))

        # Sample Configuration only few according to Robot size
        for i in range(len(v_sample)):
            s_flag = False
            for s in self.vor_samples:
                if get_distance(s, v_sample[i]) < self.max_radius:
                    s_flag = True
                    break
            if not s_flag:
                r_sample_x = round(v_sample[i][0], 2)
                r_sample_y = round(v_sample[i][1], 2)
                self.vor_samples.append((r_sample_x, r_sample_y))
        # self.vor_samples = v_sample

        return self.vor_samples

    def pick_for_task(self, robot_map, goals):
        minx, miny, maxx, maxy = self.bounds

        waypoints = []
        for key, robot in robot_map.items():
            waypoints.append(robot.pos)
        for goal in goals:
            waypoints += goal
        vor_way = []

        #  For Filter
        temp_vor_samples = []

        for vor_s in self.vor_samples:
            way_flag = False
            for way in waypoints:
                if get_distance(way, vor_s) < self.max_radius *3:
                    way_flag = True
            if not way_flag:
                temp_vor_samples.append(vor_s)

        shuffle(temp_vor_samples)

        for vor_s in temp_vor_samples:
            min_obs_dist = 10000
            vor_p = Point(vor_s)
            temp_obs = None
            obs_num = 0
            bound_num = 0

            for obs in self.obstacles:
                dist_obs = obs.distance(vor_p)
                if dist_obs < self.max_radius *3:
                    obs_num += 1
                    if dist_obs < min_obs_dist:
                        min_obs_dist = dist_obs
                        temp_obs = obs
            if vor_s[0] - minx < self.max_radius * 3:
                bound_num += 1
            elif maxx - vor_s[0] < self.max_radius * 3:
                bound_num += 1
            if vor_s[1] - miny < self.max_radius * 3:
                bound_num += 1
            elif maxy - vor_s[1] < self.max_radius * 3:
                bound_num += 1

            if obs_num == 1 and bound_num == 0:
                # vor_way.append(vor_s)
                a = list(temp_obs.boundary.coords)
                ax = [ix for (ix, _) in a]
                ay = [iy for (_, iy) in a]
                min_x = min(ax)
                min_y = min(ay)
                max_x = max(ax)
                max_y = max(ay)
                if (min_x < vor_s[0] < max_x) and (min_y < vor_s[1] < max_y):
                    vor_flag = False
                    for temp_vor in vor_way:
                        if get_distance(temp_vor, vor_s) < self.max_radius * 6: #6
                            vor_flag = True
                            break
                    if not vor_flag:
                        vor_way.append(vor_s)
            elif obs_num > 1 or (obs_num > 0 and bound_num > 0) or bound_num > 1:
                vor_flag = False
                for temp_vor in vor_way:
                    if get_distance(temp_vor, vor_s) < self.max_radius * 6: #6
                        vor_flag = True
                        break
                if not vor_flag:
                    vor_way.append(vor_s)
            # else:
            #     vor_flag = False
            #     for temp_vor in vor_way:
            #         if get_distance(temp_vor, vor_s) < self.max_radius * 6: #6
            #             vor_flag = True
            #             break
            #     if not vor_flag:
            #         vor_way.append(vor_s)

        return vor_way

    def weighted_for_task(self, robot_map, goals, samples, weight):
        waypoints = []
        for key, robot in robot_map.items():
            waypoints.append(robot.pos)
        for goal in goals:
            waypoints += goal
        weight_way = []

        #  For Filter
        temp_weight_samples = []

        for weight_s in samples:
            way_flag = False
            for way in waypoints:
                if get_distance(way, weight_s) < self.max_radius * weight:
                    way_flag = True
            if not way_flag:
                temp_weight_samples.append(weight_s)


        while(len(temp_weight_samples) > 0):
            shuffle(temp_weight_samples)
            weight_s = temp_weight_samples.pop()
            weight_way.append(weight_s)
            new_temp_weight = []
            for sample in temp_weight_samples:
                if get_distance(sample, weight_s) > self.max_radius * weight:
                    new_temp_weight.append(sample)
            temp_weight_samples = new_temp_weight

        return weight_way




