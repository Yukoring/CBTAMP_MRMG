#!/usr/bin/env python3
from matplotlib.patches import Circle, Polygon, Rectangle
from shapely.geometry import LineString, Point
import matplotlib.pyplot as plt
import numpy as np
import math
from copy import deepcopy
from utils import *
from matplotlib import animation

from descartes import PolygonPatch

plt.rcParams['animation.ffmpeg_path'] = 'ffmpeg'

Colors = ['yellow', 'orange', 'magenta', 'black', 'red', 'blue', 'green', 'white']
Robot_Colors = ['red', 'blue', 'yellowgreen', 'tan', 'slategrey', 'pink', 'magenta', 'teal']
# Robot_Colors = ['blue', 'red', 'red', 'blue', 'slategrey', 'pink', 'magenta', 'teal']
# Robot_Colors = ['red', 'blue', 'yellow', 'orange', 'red', 'blue', 'magenta', 'teal']
Goal_Colors = ['red', 'pink', 'snow', 'orange', 'blue', 'grey', 'steelblue','teal']
Task_Colors = ['royalblue', 'yellow', 'limegreen', 'darkgray', 'brown']


def plot_environment(env, t, figsize=None):
    minx, miny, maxx, maxy = env.bounds
    max_width, max_height = 24, 11
    if figsize is None:
        width, height = max_width, (maxy-miny)*max_width/(maxx-minx)
        if height > 5:
            width, height = (maxx-minx)*max_height/(maxy-miny), max_height
        figsize = (width, height)
    aspect = maxx/maxy

    f= plt.figure("Environment", figsize=(8 * aspect, 8))
    ax = f.add_subplot(111)
    ax.clear()

    for name, obs in env.obstacles_map.items():
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    for name, goal in env.goals_map.items():
        if goal['based'] == 'region':
            plot_poly(ax, goal['shape'], 'green')

    for name, robot in env.robots_map.items():
        buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
        plot_robot(ax, buffered_robot, robot.index)

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.title("Environment")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig('figure/%d_environment.png'%t)
    return ax

def plot_voronoi(env, samples, goals, t, figsize=None):
    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    f= plt.figure("Voronoi", figsize = (8 * aspect, 8))
    ax = f.add_subplot(111)
    ax.clear()

    for name, obs in env.obstacles_map.items():
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    for name, goal in env.goals_map.items():
        if goal['based'] == 'region':
            plot_poly(ax, goal['shape'], 'green')

    for name, robot in env.robots_map.items():
        buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
        plot_robot(ax, buffered_robot, robot.index)

    for i, ele in enumerate(goals):
        for goal in ele:
            buffered_goal_sample = Point(goal).buffer(0.10)
            plot_goal(ax, buffered_goal_sample, i)

    for sample in samples:
        buffered_sample = Point(sample).buffer(0.05)
        plot_poly(ax, buffered_sample, 'black')

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.title("Voronoi Samples")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig('figure/%d_voronoi.png'%t)
    return ax

def plot_prm(env, samples, roadmap, goals, t, figsize=None):
    plot_prm = {}
    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    for key, val in env.robots_map.items():
        rr = val.robot_radius
        if not rr in plot_prm:
            f = plt.figure("PRM", figsize=(8 * aspect, 8))
            ax = f.add_subplot(111)
            ax.clear()
            for name, obs in env.obstacles_map.items():
                patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
                ax.add_patch(patch)

            for name, goal in env.goals_map.items():
                if goal['based'] == 'region':
                    plot_poly(ax, goal['shape'], 'green')

            for name, robot in env.robots_map.items():
                buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
                plot_robot(ax, buffered_robot, robot.index)

            for i, ele in enumerate(goals):
                for goal in ele:
                    buffered_goal_sample = Point(goal).buffer(0.10)
                    plot_goal(ax, buffered_goal_sample, i)

            for sample in samples[rr]:
                buffered_sample = Point(sample).buffer(0.05)
                plot_poly(ax, buffered_sample, 'black')

            for i, first_sample in enumerate(roadmap[rr]):
                for second_sample in first_sample:
                    line = LineString([samples[rr][i], samples[rr][second_sample]])
                    plot_line(ax, line)

            plt.title("Probabilistic Roadmap for robot radius %0.2f"%rr)
            plt.xlim([minx, maxx])
            plt.ylim([miny, maxy])
            ax.set_aspect('equal', adjustable='box')
            plot_prm[rr] = ax
            plt.savefig('figure/%d_PRM_%0.2f.png'%(t, rr))

    return plot_prm

def plot_obj_check(env, makeprob, goals, count, t, figsize=None):
    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    f= plt.figure("obj_check", figsize = (8 * aspect, 8))
    ax = f.add_subplot(111)
    ax.clear()

    for name, obs in env.obstacles_map.items():
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    for name, goal in env.goals_map.items():
        if goal['based'] == 'region':
            plot_poly(ax, goal['shape'], 'green')

    for name, robot in env.robots_map.items():
        buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
        plot_robot(ax, buffered_robot, robot.index)
    for i, ele in enumerate(goals):
        for goal in ele:
            buffered_goal_sample = Point(goal).buffer(0.10)
            plot_goal(ax, buffered_goal_sample, i)

    way_list = makeprob.obj_ins['waypoint']
    for i in range(len(way_list)-1):
        connect = way_list[i].connected
        loc_wp1 = way_list[i].loc
        for j in range(i+1, len(way_list)):
            if way_list[j].ind in connect:
                loc_wp2 = way_list[j].loc
                line = LineString([loc_wp1, loc_wp2])
                plot_line(ax, line)

    plot_text(ax, makeprob.obj_ins)

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.title("Task Domain View")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig('figure/%d_%dth_3_obj_check.png'%(t,count))
    return ax

def plot_plan(env, makeprob, goals, robot_path, count, domain, t, figsize=None):
    plot_ax = {}

    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    for key, val in env.robots_map.items():
        rr = val.robot_radius
        if not rr in plot_ax:
            f = plt.figure("%0.2f_plan"%rr, figsize=(8 * aspect, 8))
            ax = f.add_subplot(111)
            ax.clear()

            for name, obs in env.obstacles_map.items():
                patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
                ax.add_patch(patch)

            for name, goal in env.goals_map.items():
                if goal['based'] == 'region':
                    plot_poly(ax, goal['shape'], 'green')

            for name, robot in env.robots_map.items():
                buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
                plot_robot(ax, buffered_robot, robot.index)

            for i, ele in enumerate(goals):
                for goal in ele:
                    buffered_goal_sample = Point(goal).buffer(0.10)
                    plot_goal(ax, buffered_goal_sample, i)

            # for sample in makeprob.samples[rr]:
            #     buffered_sample = Point(sample).buffer(0.05)
            #     plot_poly(ax, buffered_sample, 'black')

            # for i, first_sample in enumerate(makeprob.roadmaps[rr]):
            #     for second_sample in first_sample:
            #         line = LineString([makeprob.samples[rr][i], makeprob.samples[rr][second_sample]])
            #         plot_line(ax, line)
            plot_text(ax, makeprob.obj_ins)

            plt.xlim([minx, maxx])
            plt.ylim([miny, maxy])
            plt.title("Plan for %0.2f"%rr)
            ax.set_aspect('equal', adjustable='box')
            plot_ax[rr] = ax
        if key in robot_path:
            path = robot_path[key]['path_only']
            # print(path)
            if len(path) > 1:
                plot_path(plot_ax[rr], path, rr, val.index)
        if domain == 0:
            plt.savefig('figure/%d_%dth_4_early_plan_%0.2f.png'%(t, count,rr))
        elif domain == 1:
            plt.savefig('figure/%d_%dth_4_late_plan_%0.2f.png'%(t, count, rr))
        elif domain == 2:
            plt.savefig('figure/%d_%dth_4_const_plan_%0.2f.png'%(t, count,rr))
        elif domain == 3:
            plt.savefig('figure/%d_%dth_4_prioritized_plan_%0.2f.png'%(t, count,rr))

    return plot_ax

def plot_best_plan(env, obj, goals, robot_path, t, figsize=None):
    plot_ax = {}

    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    for key, val in env.robots_map.items():
        rr = val.robot_radius
        if not rr in plot_ax:
            f = plt.figure("%0.2f_plan"%rr, figsize=(8 * aspect, 8))
            ax = f.add_subplot(111)
            ax.clear()
            for name, obs in env.obstacles_map.items():
                patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
                ax.add_patch(patch)

            for name, goal in env.goals_map.items():
                if goal['based'] == 'region':
                    plot_poly(ax, goal['shape'], 'green')

            for name, robot in env.robots_map.items():
                buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
                plot_robot(ax, buffered_robot, robot.index)

            for i, ele in enumerate(goals):
                for goal in ele:
                    buffered_goal_sample = Point(goal).buffer(0.10)
                    plot_goal(ax, buffered_goal_sample, i)

            plot_text(ax, obj)

            plt.xlim([minx, maxx])
            plt.ylim([miny, maxy])
            plt.title("Plan for %0.2f"%rr)
            ax.set_aspect('equal', adjustable='box')
            plot_ax[rr] = ax
        if key in robot_path:
            path = robot_path[key]['path_only']
            if len(path) > 1:
                plot_path(plot_ax[rr], path, rr, val.index)
        plt.savefig('figure/%d_best_plan_%0.2f.png'%(t,rr))
    return plot_ax

def plot_collision_update(env, prm, col_env, goals, count, t, figsize=None):
    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    f = plt.figure("Collision Update", figsize = (8* aspect, 8))
    ax = f.add_subplot(111)
    ax.clear()

    samples = prm.samples[prm.max_rr]
    roadmap = prm.roadmaps[prm.max_rr]

    # Goal Sample
    goal_samples = []
    for ele in goals:
        goal_samples += ele

    for name, obs in env.obstacles_map.items():
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    for name, goal in env.goals_map.items():
        if goal['based'] == 'region':
            plot_poly(ax, goal['shape'], 'green')

    for name, robot in env.robots_map.items():
        buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
        plot_robot(ax, buffered_robot, robot.index)
    for i, ele in enumerate(goals):
        for goal in ele:
            buffered_goal_sample = Point(goal).buffer(0.10)
            plot_goal(ax, buffered_goal_sample, i)

    # for sample in samples:
    #     buffered_sample = Point(sample).buffer(0.05)
    #     plot_poly(ax, buffered_sample, 'black')

    # for i, first_sample in enumerate(roadmap):
    #     for second_sample in first_sample:
    #         line = LineString([samples[i], samples[second_sample]])
    #         plot_line(ax, line)

    for col in col_env:
        entry = col[0]
        col_samples = col[1]
        col_index = col[2]
        for col_sample in col_samples:
            buffered_col_sample = Point(col_sample).buffer(0.2)
            if col_sample in goal_samples:
                plot_poly(ax, buffered_col_sample, 'lightgreen')
            else:
                plot_poly(ax, buffered_col_sample, 'deeppink')
        if col_index == 0:
            for ele in entry:
                buffered_end = Point(ele[0]).buffer(0.2)
                plot_poly(ax, buffered_end, 'yellow')
                if len(ele) == 2:
                    buffered_entry = Point(ele[1]).buffer(0.2)
                    # plot_poly(ax, buffered_entry, 'indigo')
                    plot_poly(ax, buffered_entry, 'seagreen')
                elif len(ele) == 3:
                    buffered_entry1 = Point(ele[1]).buffer(0.2)
                    buffered_entry2 = Point(ele[2]).buffer(0.2)
                    plot_poly(ax, buffered_entry1, 'seagreen')
                    plot_poly(ax, buffered_entry2, 'seagreen')
                elif len(ele) > 3:
                    for i in range(1, len(ele)):
                        buffered_entry = Point(ele[i]).buffer(0.2)
                        # plot_poly(ax, buffered_entry, 'salmon')
                        plot_poly(ax, buffered_entry, 'seagreen')
        else:
            for ele in entry:
                buffered_end = Point(ele[0]).buffer(0.2)
                plot_poly(ax, buffered_end, 'dodgerblue')

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.title("Collision Construction")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig('figure/%d_%dth_2_col_update.png'%(t,count))
    return ax

def plot_collision(env, prm, collision, goals, count, t, figsize=None):
    minx, miny, maxx, maxy = env.bounds

    aspect = maxx/maxy

    f= plt.figure("Collisions", figsize = (8 * aspect, 8))
    ax = f.add_subplot(111)
    ax.clear()

    samples = prm.samples[prm.max_rr]
    roadmap = prm.roadmaps[prm.max_rr]

    for name, obs in env.obstacles_map.items():
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    for name, goal in env.goals_map.items():
        if goal['based'] == 'region':
            plot_poly(ax, goal['shape'], 'green')

    for name, robot in env.robots_map.items():
        buffered_robot = Point(robot.pos).buffer(robot.robot_radius)
        plot_robot(ax, buffered_robot, robot.index)
    for i, ele in enumerate(goals):
        for goal in ele:
            buffered_goal_sample = Point(goal).buffer(0.10)
            plot_goal(ax, buffered_goal_sample, i)
    for sample in samples:
        buffered_sample = Point(sample).buffer(0.05)
        plot_poly(ax, buffered_sample, 'black')

    for i, first_sample in enumerate(roadmap):
        for second_sample in first_sample:
            line = LineString([samples[i], samples[second_sample]])
            plot_line(ax, line)
    for col in collision:
        col_samples = col[0]
        col_index = col[1]
        if col_index == 0:
            for col_sample in col_samples:
                buffered_col_sample = Point(col_sample).buffer(0.2)
                plot_poly(ax, buffered_col_sample, 'deeppink')
        else:
            for col_sample in col_samples:
                buffered_col_sample = Point(col_sample).buffer(0.2)
                plot_poly(ax, buffered_col_sample, 'dodgerblue')

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.title("Just Check")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig('figure/%d_%dth_1_col_check.png'%(t, count))
    return ax

def plot_text(ax, instance):
    #print(instance)
    for name, inst in instance.items():
        for ele in inst:
            if name == 'job':
                continue
            elif name == 'task':
                continue
            elif name == 'robot':
                ax.text(ele.loc[0], ele.loc[1] - 0.6, ele.ind, horizontalalignment='center',
                    verticalalignment='center', fontsize= 10, color = 'red')
            elif ele.loc:
                ax.text(ele.loc[0], ele.loc[1] + 0.2, ele.ind, horizontalalignment='center',
                    verticalalignment='center', fontsize= 8)

def plot_line(ax, line):
    x, y = line.xy
    ax.plot(x, y, color='gray', linewidth=1, solid_capstyle='round', zorder=1)

def plot_line_color(ax, line, color):
    x, y = line.xy
    ax.plot(x, y, color=color, linewidth=3, solid_capstyle='round', zorder=1)

def plot_linestring(ax, line, color):
    line = LineString(line)
    plot_line_color(ax, line, color)

def plot_linestring_buffer(ax, line, color):
    line = LineString(line).buffer(0.3)
    plot_poly(ax, line, color)

def plot_poly(ax, poly, color, alpha=1.0, zorder=1):
    patch = PolygonPatch(poly, fc=color, ec="black", alpha=alpha, zorder=zorder)
    ax.add_patch(patch)

def plot_goal(ax, goal, ind, alpha=1.0, zorder=1):
    patch = PolygonPatch(goal, fc=Goal_Colors[ind % len(Goal_Colors)], ec='black', alpha=alpha, zorder=zorder)
    ax.add_patch(patch)

def plot_robot(ax, robot, ind, alpha=1.0, zorder=1):
    patch = PolygonPatch(robot, fc=Robot_Colors[ind % len(Robot_Colors)], ec='black', alpha=alpha, zorder=zorder)
    ax.add_patch(patch)

def plot_path(env_plot, path, object_radius, i):
    # Plots path by taking an enviroment plot and ploting in red the edges that form part of the path
    line = LineString(path)
    x, y = line.xy
    env_plot.plot(x, y, color=Colors[i % len(Colors)], linewidth=3, solid_capstyle='round', zorder=1)

def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_dwa(ax, env, goal_region, x, pre_traj, paths):
    plt.cla()
    minx, miny, maxx, maxy = env.bounds

    for i, obs in enumerate(env.obstacles):
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)


    for goal in goal_region:
        plot_poly(ax, goal, 'green')
    for i in range(env.num_of_robot):
        buffered_robot = Point((x[i][0],x[i][1])).buffer(env.robots[i].robot_radius)
        plot_arrow(x[i][0], x[i][1], x[i][2])
        plot_path(ax, paths[i], env.robots[i].robot_radius, i)
        plot_poly(ax, buffered_robot, Robot_Colors[i%len(Robot_Colors)])
        ax.plot(pre_traj[i][:,0], pre_traj[i][:, 1], "-g")
        ax.plot(x[i][0], x[i][1], "xb")

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    ax.set_aspect('equal', adjustable='box')
    ax.axis("equal")
    ax.grid(True)

class Animation:
    def __init__(self, env, obj_ins, goals, robot_path, v_text):
        self.env = env
        self.obj_ins = obj_ins
        self.goals = goals
        self.robots = env.robots_map
        self.robot_path = robot_path
        self.goals_map = env.goals_map
        # print(robot_path)


        # Figure Size 2가지 경우
        minx, miny, maxx, maxy = env.bounds
        # max_instance = max(self.env.num_of_robot, self.env.num_of_goal)
        y_bound = maxy + 1.5
        # y_bound = 0.7*(max_instance) + maxy - 1.0

        aspect = (maxx+2)/y_bound

        #self.fig = plt.figure(frameon=False, figsize=figsize)
        self.fig = plt.figure(frameon=False, figsize = (8 * aspect, 8.25)) #8
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        self.agent_service = dict()
        self.agent_service_order = dict()
        self.task_patches = dict()
        self.goal_names = dict()

        # label
        self.label_agent_names = dict()
        # self.label_agent_type = dict()
        # self.label_goal_names = dict()
        self.label_goal_service = dict()
        self.time_text = None
        self.total_cost_text = None
        self.makespan_text = None

        self.path_with_time = dict()

        self.T = 0

        plt.xlim([minx-1, maxx+1])
        plt.ylim([miny, y_bound])
        # plt.title(" Animation")
        # Add Obstacles and Bound
        self.patches.append(Rectangle((minx, miny), maxx - minx, maxy - miny, facecolor='none', edgecolor='gray'))
        for name, obs in env.obstacles_map.items():
            self.patches.append(PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20))

        # Add Goal instance
        # Square, Star, Circle, Triangle, diamond
        total_task = []
        for name,goal in self.goals_map.items():
            for service in goal['service']:
                if not service in total_task:
                    total_task.append(service)

        for i, (name, goal) in enumerate(self.goals_map.items()):
            self.task_patches[name] = dict()
            # Goal Region and Task
            if goal['based'] == 'region':
                self.patches.append(PolygonPatch(goal['shape'], fc='green', ec='black', alpha=1.0, zorder=1))
            # Task instance- order ?
            bminx, bminy, bmaxx, bmaxy = goal['shape'].bounds
            task_ind = [[0,ele] for ele in goal['service']]
            temp_ord = deepcopy(goal['order'])
            while len(temp_ord) > 0:
                temp_name = []
                temp_remove = []
                for ele in temp_ord:
                    if not ele[0] in temp_name:
                        temp_name.append(ele[0])
                    if not ele[1] in temp_name:
                        temp_name.append(ele[1])
                for ele in temp_ord:
                    if ele[1] in temp_name:
                        temp_name.remove(ele[1])
                        for ele2 in task_ind:
                            if ele2[1] == ele[1]:
                                ele2[0] += 1
                for ele in temp_ord:
                    for ele2 in temp_name:
                        if ele2 in ele:
                            temp_remove.append(ele)
                for ele in temp_remove:
                    if ele in temp_ord:
                        temp_ord.remove(ele)
            #
            prev_ind = 0
            std_x = bmaxx + 0.3
            std_y = bmaxy + 0.3
            task_ind.sort()
            for ele in task_ind:
                if prev_ind == ele[0]:
                    std_y -= 0.3
                else:
                    std_x += 0.3
                    std_y = bmaxy
                    prev_ind = ele[0]
                task_shape = total_task.index(ele[1])
                if task_shape == 0: #Square
                    self.task_patches[name][ele[1]] = Rectangle((std_x-0.1, std_y-0.1), 0.2, 0.2, fc='royalblue', ec='black')
                elif task_shape == 1: #Star
                    sx = [std_x,std_x-0.025,std_x-0.1,std_x-0.025,std_x,std_x+0.025,std_x+0.1,std_x+0.025]
                    sy = [std_y+0.1,std_y+0.025,std_y,std_y-0.025,std_y-0.1,std_y-0.025,std_y,std_y+0.025]
                    self.task_patches[name][ele[1]] = Polygon(xy=list(zip(sx,sy)), fc='yellow', ec='black')
                elif task_shape == 2: #Circle
                    self.task_patches[name][ele[1]] = Circle((std_x,std_y), 0.1, fc='limegreen', ec='black')
                elif task_shape == 3: #Triangle
                    sx = [std_x, std_x-0.1, std_x+0.1]
                    sy = [std_y+0.13, std_y-0.07, std_y-0.07]
                    self.task_patches[name][ele[1]] = Polygon(xy=list(zip(sx,sy)), fc='darkgray', ec='black')
                elif task_shape == 4: #Diamond
                    sx = [std_x, std_x-0.05, std_x, std_x+0.05]
                    sy = [std_y+0.1, std_y, std_y - 0.1, std_y]
                    self.task_patches[name][ele[1]] = Polygon(xy=list(zip(sx,sy)), fc='brown', ec='black')
                self.patches.append(self.task_patches[name][ele[1]])

            for goal in self.goals[i]:
                buffered_goal_sample = Point(goal).buffer(0.10)
                self.patches.append(PolygonPatch(buffered_goal_sample, fc = Goal_Colors[i % len(Goal_Colors)], ec='black', alpha=1.0, zorder=1.0))

        # Add Path
        # for name, ele in self.robots.items():
        #     ind = ele.index
        #     local_path = ele.path['path_only']
        #     if len(local_path) > 1:
        #         buffered_line = LineString(local_path).buffer(0.03)
        #         line_color = Colors[ind % len(Colors)]
        #         self.patches.append(PolygonPatch(buffered_line, fc = line_color, ec= line_color, alpha=1.0, zorder=1.0))

        # Add Agent
        for name, ele in self.robots.items():
            self.agent_service[name] = dict()
            self.agent_service_order[name] = []
            self.agents[name] = Circle((ele.pos[0], ele.pos[1]), ele.robot_radius, facecolor=Robot_Colors[i % len(Robot_Colors)],
                                    edgecolor='black')
            self.agents[name].original_face_color = Robot_Colors[ele.index % len(Robot_Colors)]
            self.patches.append(self.agents[name])
            self.T = max(self.T, self.robots[name].path_time)

            self.agent_names[name] = self.ax.text(ele.pos[0], ele.pos[1] + 0.4, name, fontsize = 10)
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

            temp_agent_task = []
            for i in range(len(ele.service)):
                temp_agent_task.append([ele.action_cost[i],ele.service[i]])
            temp_agent_task.sort()

            for i, ele2 in enumerate(temp_agent_task):
                std_x = ele.pos[0] - 0.3 + i*0.3
                std_y = ele.pos[1] - 0.5
                task_shape = total_task.index(ele2[1])
                self.agent_service_order[name].append(task_shape)
                if task_shape == 0: #Square
                    self.agent_service[name][ele2[1]] = Rectangle((std_x-0.1, std_y-0.1), 0.2, 0.2, fc='royalblue', ec='black')
                elif task_shape == 1: #Star
                    sx = [std_x,std_x-0.025,std_x-0.1,std_x-0.025,std_x,std_x+0.025,std_x+0.1,std_x+0.025]
                    sy = [std_y+0.1,std_y+0.025,std_y,std_y-0.025,std_y-0.1,std_y-0.025,std_y,std_y+0.025]
                    self.agent_service[name][ele2[1]] = Polygon(xy=list(zip(sx,sy)), fc='yellow', ec='black')
                elif task_shape == 2: #Circle
                    self.agent_service[name][ele2[1]] = Circle((std_x,std_y), 0.1, fc='limegreen', ec='black')
                elif task_shape == 3: #Triangle
                    sx = [std_x, std_x-0.1, std_x+0.1]
                    sy = [std_y+0.13, std_y-0.07, std_y-0.07]
                    self.agent_service[name][ele2[1]] = Polygon(xy=list(zip(sx,sy)), fc='darkgray', ec='black')
                elif task_shape == 4: #Diamond
                    sx = [std_x, std_x-0.05, std_x, std_x+0.05]
                    sy = [std_y+0.1, std_y, std_y - 0.1, std_y]
                    self.agent_service[name][ele2[1]] = Polygon(xy=list(zip(sx,sy)), fc='brown', ec='black')
                self.patches.append(self.agent_service[name][ele2[1]])
                self.agent_service[name][ele2[1]].original_face_color = Task_Colors[task_shape]

        # Add Obj Ins
        for name, ele in self.obj_ins.items():
            for ins in ele:
                if name == 'waypoint':
                    if ins.instype == 'normal_way':
                        way_name = self.ax.text(ins.loc[0], ins.loc[1] + 0.20, ins.ind, fontsize = 8)
                        way_name.set_horizontalalignment('center')
                        way_name.set_verticalalignment('center')
                        self.artists.append(way_name)
                    else:
                        buffered_way = Point(ins.loc).buffer(0.05)
                        self.patches.append(PolygonPatch(buffered_way, fc = 'pink', ec='black', alpha=1.0, zorder=1.0))
                        way_name = self.ax.text(ins.loc[0], ins.loc[1] + 0.15, ins.ind, fontsize = 8)
                        way_name.set_horizontalalignment('center')
                        way_name.set_verticalalignment('center')
                        self.artists.append(way_name)
        # Add Time Text
        self.time_text = self.ax.text(maxx-0.1, y_bound-0.5, "Time: 0 sec")
        self.time_text.set_horizontalalignment('right')
        self.time_text.set_verticalalignment('center')
        self.time_text.set_fontweight('bold')
        self.artists.append(self.time_text)

        # title_name = "Map with " + str(self.env.num_of_goal) + " Goals and " + str(self.env.num_of_robot) + " Robots"
        title_name = "Every Action (EA)"
        self.title_text = self.ax.text((maxx+minx)/2, y_bound-0.5, title_name, fontsize = 'large')
        self.title_text.set_horizontalalignment('center')
        self.title_text.set_verticalalignment('center')
        self.title_text.set_fontweight('bold')
        self.artists.append(self.title_text)


        make_span = "MakeSpan: " + str(v_text[0])
        self.makespan_text = self.ax.text(maxx-0.1, y_bound-0.9, make_span)
        self.makespan_text.set_horizontalalignment('right')
        self.makespan_text.set_verticalalignment('center')
        self.makespan_text.set_fontweight('bold')
        self.artists.append(self.makespan_text)

        total = "Robot Total Cost: " + str(v_text[1])
        self.total_cost_text = self.ax.text(maxx-0.1, y_bound-1.3, total)
        self.total_cost_text.set_horizontalalignment('right')
        self.total_cost_text.set_verticalalignment('center')
        self.total_cost_text.set_fontweight('bold')
        self.artists.append(self.total_cost_text)

        # for name, ele in self.robots.items():
        #     robot_ind = ele.index
        #     robot_text = name + ":"
        #     for i in range(len(ele.service)):
        #         robot_text = robot_text + " " + ele.service[i] + " (" + str(ele.action_cost[i]) + ")"
        #     self.label_agent_names[name] = self.ax.text(maxx-0.1, y_bound - 1.7 - (0.4*robot_ind), robot_text, fontsize = 'small')
        #     self.label_agent_names[name].set_horizontalalignment('right')
        #     self.label_agent_names[name].set_verticalalignment('center')
        #     self.artists.append(self.label_agent_names[name])

        self.work_var = None

        # Add Goal Text
        for name, ele in self.goals_map.items():
            goal_center = ele['samples'][0]
            ext = list(ele['shape'].exterior.coords)
            upper_y = -100000
            for a in ext:
                upper_y = max(upper_y, a[1])
            # goal_index = ele['index']
            # a_name = name+':'
            # self.label_goal_names[name] = self.ax.text(minx+0.0, y_bound- 1.0-(0.4*goal_index), a_name)
            # self.label_goal_names[name].set_horizontalalignment('left')
            # self.label_goal_names[name].set_verticalalignment('center')
            # self.artists.append(self.label_goal_names[name])
            if ele['based'] == 'obstacle':
                self.goal_names[name] = self.ax.text(ele['shape'].centroid.coords[0][0], ele['shape'].centroid.coords[0][1], name, fontsize = 10)
            else:
                self.goal_names[name] = self.ax.text(goal_center[0], upper_y+0.2, name, fontsize = 10)
            self.goal_names[name].set_horizontalalignment('center')
            self.goal_names[name].set_verticalalignment('center')
            self.artists.append(self.goal_names[name])

            # self.label_goal_service[name] = dict()
            # for i, service_name in enumerate(ele['service']):
            #     ########## new world
            #     # self.label_goal_service[name][service_name] = self.ax.text(minx+1.6+(1.0*i), y_bound- 1.0 -(0.4*goal_index), service_name)
            #     # self.label_goal_service[name][service_name].set_horizontalalignment('left')
            #     # self.label_goal_service[name][service_name].set_verticalalignment('center')
            #     self.label_goal_service[name][service_name] = list()
            #     self.label_goal_service[name][service_name].append(self.ax.text(minx+1.2+(0.8*i), y_bound- 1.0 -(0.4*goal_index), service_name))
            #     # self.label_goal_service[name][service_name] = self.ax.text(minx+1.0+(0.8*i), y_bound- 1.0 -(0.4*goal_index), service_name)
            #     self.label_goal_service[name][service_name][0].set_horizontalalignment('left')
            #     self.label_goal_service[name][service_name][0].set_verticalalignment('center')
            #     self.label_goal_service[name][service_name].append(0)
            #     self.artists.append(self.label_goal_service[name][service_name][0])

        for name, ele in self.robot_path.items():
            cost_path = ele['path_cost']
            temp_path = []
            temp_time = 0.0
            path_len = len(cost_path)
            starts = None
            if path_len > 1:
                for i in range(1, path_len):
                    if i == 1:
                        if type(cost_path[i-1]) == list:
                            for partial in cost_path[i-1][2]:
                                if partial[1] == 2:
                                    temp_path.append((cost_path[i-1][0], temp_time))
                                    temp_time += round(partial[0], 2)
                                else:
                                    temp_path.append((cost_path[i-1][0], temp_time, partial[1], partial[2]))
                                    temp_time += round(partial[0], 2)
                            temp_path.append((cost_path[i-1][0], temp_time))
                            starts = cost_path[i-1][0]
                        else:
                            temp_path.append((cost_path[i-1], temp_time))
                            starts = cost_path[i-1]
                    if type(cost_path[i]) == list:
                        temp_time += round(get_distance(starts, cost_path[i][0]),2)
                        temp_path.append((cost_path[i][0], temp_time))
                        for partial in cost_path[i][2]:
                            if partial[1] == 2:
                                temp_time += round(partial[0], 2)
                                temp_path.append((cost_path[i][0], temp_time))
                            else:
                                temp_time += round(partial[0], 2)
                                temp_path.append((cost_path[i][0], temp_time, partial[1], partial[2]))
                        starts = cost_path[i][0]
                    else:
                        temp_time += round(get_distance(starts, cost_path[i]),2)
                        temp_path.append((cost_path[i], temp_time))
                        starts = cost_path[i]
                self.path_with_time[name] = temp_path
            else:
                self.path_with_time[name] = [(cost_path[0][0], temp_time)]

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 3) * 10,
                                                 interval=80,
                                                 blit=True)

    def save(self, file_name, speed):
        writervideo = animation.FFMpegWriter(fps = 10 * speed)
        self.animation.save(
            file_name,
            writer = writervideo
        )

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        # Init Goal position when t = 0
        if t ==0:
            pass
        set_time = str(round(t/10, 3))
        self.time_text.set_text("Time: " + set_time + " sec")

        # reset all colors
        for name, ele in self.agent_service.items():
            for name2, ele2 in ele.items():
                ele2.set_facecolor(ele2.original_face_color)

        for name, ele in self.path_with_time.items():
            pos = self.get_state_time(t / 10, ele, name)
            self.agents[name].center = (pos[0], pos[1])
            self.agent_names[name].set_position((pos[0], pos[1] + 0.4))
            for i, (name2, ele2) in enumerate(self.agent_service[name].items()):
                std_x = pos[0] - 0.5 + i*0.3
                std_y = pos[1] - 0.5
                task_shape = self.agent_service_order[name][i]
                if task_shape == 0: #Square
                     ele2.set_xy((std_x-0.1, std_y-0.1))
                elif task_shape == 1: #Star
                    sx = [std_x,std_x-0.025,std_x-0.1,std_x-0.025,std_x,std_x+0.025,std_x+0.1,std_x+0.025]
                    sy = [std_y+0.1,std_y+0.025,std_y,std_y-0.025,std_y-0.1,std_y-0.025,std_y,std_y+0.025]
                    ele2.set_xy(xy=list(zip(sx,sy)))
                elif task_shape == 2: #Circle
                    ele2.center = (std_x, std_y)
                elif task_shape == 3: #Triangle
                    sx = [std_x, std_x-0.1, std_x+0.1]
                    sy = [std_y+0.13, std_y-0.07, std_y-0.07]
                    ele2.set_xy(xy=list(zip(sx,sy)))
                elif task_shape == 4: #Diamond
                    sx = [std_x, std_x-0.05, std_x, std_x+0.05]
                    sy = [std_y+0.1, std_y, std_y - 0.1, std_y]
                    ele2.set_xy(xy=list(zip(sx,sy)))


        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                rr1 = d1.get_radius()
                rr2 = d2.get_radius()
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < rr1+rr2:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    #@staticmethod
    def get_state_time(self, t, path, r_name):
        if t <= path[0][1]:
            self.work_var = None
            return np.array(path[0][0])
        elif t >= path[-1][1]:
            self.work_var = None
            if len(path[-1]) == 4:
                self.task_patches[path[-1][3]][path[-1][2]].set_visible(False)
                # self.label_goal_service[path[-1][3]][path[-1][2]][1] = 2
            return np.array(path[-1][0])
        else:
            for i, time_ind in enumerate(path):
                if t < time_ind[1]:
                    i = i-1
                    break
            x1, y1 = path[i][0]
            x2, y2 = path[i+1][0]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)

            if len(path[i]) == 4:
                self.task_patches[path[i][3]][path[i][2]].set_visible(False)
                # self.label_goal_service[path[i][3]][path[i][2]][1] = 2
            if len(path[i+1]) == 4:
                self.task_patches[path[i+1][3]][path[i+1][2]].set_facecolor('red')
                self.agent_service[r_name][path[i+1][2]].set_facecolor('red')
                # self.label_goal_service[path[i+1][3]][path[i+1][2]][1] = 1
            if dx ==0 and dy == 0:
                if len(path[i+1]) ==4:
                    self.work_var = (path[i+1][2], path[i+1][3])
                else:
                    self.work_var = None
                return (x2, y2)
            pos_x = x1 + (t - path[i][1]) * 1 * math.cos(yaw)
            pos_y = y1 + (t - path[i][1]) * 1 * math.sin(yaw)
            pos = (pos_x, pos_y)
            self.work_var = None
            return pos

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos
