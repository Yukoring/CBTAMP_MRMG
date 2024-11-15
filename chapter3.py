#!/usr/bin/python3
import argparse
import time
import os
import glob
import subprocess
from pathlib import Path
from copy import deepcopy
from environment import Environment
from prm_chapter3 import PRMPlanning
from pddlgenerator_chapter3 import PDDLProblemGenerator, read_domain
from voronoi import VoronoiPlanning
from visualize import *
from prioritized import PrioritizedPlanningSolver
from collision import *

if __name__ == '__main__':
    """
    Argument Management
    Map: Yaml File (Robot Config, Goals, Env (obstacle))
    Domain PDDL: Domain File (Weld-Bolt)
    Problem PDDL: File name generated from module
    TPWE strategy: simple, weighted, goal, vor.
    """
    parser = argparse.ArgumentParser(description='Runs Task and Motion Planning Module')
    parser.add_argument('-m', '--map', type=str, default = None,
                        help = 'The name of an instance file')
    parser.add_argument('-d', '--domain', type=str, default = None,
                        help = 'The name of input domain file')
    parser.add_argument('-p', '--problem', type=str, default = 'example_problem.pddl',
                        help = 'The name of output problem file')
    parser.add_argument('-s', '--strategy', type=str, default = 'simple',
                        help = 'The name of solver')

    parser.add_argument('-pl', '--plot', action='store_true', default = False,
                        help = 'Use batch output instead of plot')
    parser.add_argument('-t', '--trial', type=int, default = 1,
                        help = 'Experiment Number')
    args = parser.parse_args()

    dFile = args.domain
    mFile = args.map
    pFile = args.problem
    strategy = args.strategy

    plot = args.plot
    trial = args.trial

    additional_domain = True

    # Domain, Problem File Setting
    d_name = read_domain(dFile)

    # Visualization Setting
    env_plot = None
    prm_plot = None
    plan_plot = None
    collision_plot = None
    obj_plot = None
    strategy_plot = None

    # CSV File Setting
    result_file = open("results.csv", "w", buffering=1)

    figs = glob.glob('figure/*')
    for f in figs:
        try:
            os.remove(f)
        except OSError as e:
            print("Error: %s : %s" % (f, e.strerror))

    pddls = glob.glob('pddl_prob_plan/*.pddl')
    for f in pddls:
        try:
            os.remove(f)
        except OSError as e:
            print("Error: %s : %s"% (f, e.strerror))

    # Program Start
    for t in range(1, trial+1):
        print("----------------------------------------------------------------")
        print("%dth trial start"%t)

        # Time Initialize
        roadmap_time = 0
        taskplan_time = 0
        motionplan_time = 0
        probmake_time = 0

        # Load Environment from yaml File
        # and Visualize
        start_time = time.time()
        test_time = time.time()
        env = Environment(mFile)
        if plot:
            env_plot = plot_environment(env, t)

        # Voronoi 같은경우는 sample을 prm에 넣어야하기때문에 먼저 선택

        # Voronoi Sampling
        v_sample = []
        if strategy == 'vor':
            vor = VoronoiPlanning(env)
            v_sample = vor.voronoi_sampling()
        elif strategy == 'weighted':
            vor = VoronoiPlanning(env)

        """
        PRM Generate Part from map file
        Samples [it can be used for waypoint in problem file]
        Roadmap [it can be used for edge in problem file]
        Goal_Sample [it must be used for waypoint and job located in problem file]
        """
        prm = PRMPlanning(env, v_sample, strategy)
        init_samples, init_roadmap, goal_samples, parking = prm.ConstructPhase()
        roadmap_time = time.time() - test_time

        vor_way = parking
        if strategy == 'vor':
            vor_way = vor.pick_for_task(env.robots_map, goal_samples)
            prm.vor_ways = vor_way
        elif strategy == 'weighted':
            weight = 5.0
            vor_way = vor.weighted_for_task(env.robots_map, goal_samples, prm.max_samples, weight)

        # Visualizing PRM
        if plot:
            if strategy == 'vor':
                strategy_plot = plot_voronoi(env, vor_way, goal_samples, t)
                # voronoi_plot = plot_voronoi(env, v_sample, goal_samples, t)
            elif strategy == 'weighted':
                strategy_plot = plot_voronoi(env, vor_way, goal_samples, t)
            prm_plot = plot_prm(env, init_samples, init_roadmap, goal_samples, t)

        """
        Task Planning Domain
        """
        count = 0

        test_time = time.time()
        makeprob = PDDLProblemGenerator(env, prm, vor_way, d_name, pFile)

        makeprob.env_instance()

        pName = makeprob.generate_problem(t)
        elapsed_time = time.time() - test_time
        probmake_time += elapsed_time

        if plot:
            obj_plot = plot_obj_check(env, makeprob, goal_samples, count, t)

        planFile = 'pddl_prob_plan/%d_plan%d.pddl'%(t, count)
        planFile_c = 'pddl_prob_plan/%d_plan%d_c.pddl'%(t, count)
        best_cost = 1000000
        best_robot_path = None
        best_total_cost = 1000000
        best_index = [None, None] # count, Early or Late or Const
        best_obj_ins = None
        best_col_env = None

        test_time = time.time()
        command = './planners/optic-cplex-wrapper ' + dFile + ' ' + pName + ' ' + planFile
        popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        (stdoutdata, stderrdata) = popen.communicate()

        elapsed_time = time.time() - test_time
        taskplan_time += elapsed_time

        plan_f = Path(planFile)
        if not plan_f.is_file():
            print("Mission Failed")
            obj_plot = plot_obj_check(env, makeprob, goal_samples, -1, t)
            continue

        test_time = time.time()
        init_plan_dispatch, init_task_cost, init_cal_timem, prioritized_plan = makeprob.parse_pddl_plan(planFile)

        prior = PrioritizedPlanningSolver(prm.max_roadmap, prm.max_samples, prioritized_plan)
        robot_path, makespan, total_cost = prior.find_solution()

        elapsed_time = time.time() - test_time
        motionplan_time += elapsed_time

        best_cost = makespan
        best_robot_path = robot_path
        best_total_cost = total_cost
        best_obj_ins = deepcopy(makeprob.obj_ins)

        if plot:
            plan_plot = plot_plan(env, makeprob, goal_samples, robot_path, count, 3, t)


        if not best_robot_path:
            print("Mission Failed")
            obj_plot = plot_obj_check(env, makeprob, goal_samples, count, t)
            result_file.write("{},{}\n".format(t, "Failed"))
            continue


        endtime = time.time() - start_time
        print("Mission Complete")

        print("Roadmap Construct Time: ", roadmap_time)
        print("Task Planning Time: ", taskplan_time)
        print("Problem Making Time: ", probmake_time)
        print("Motion Planning Time: ", motionplan_time)

        print("Total Time: ", endtime)

        if best_index[1] == 0:
            print(best_index[0], "th Early Plan Selected!, Makespan: ", best_cost, " Total Cost: ", best_total_cost)
        elif best_index[1] == 1:
            print(best_index[0], "th Late Plan Selected!, Makespan: ", best_cost, " Total Cost: ", best_total_cost)
        elif best_index[1] == 2:
            print(best_index[0], "th Constraint Plan Selected!, Makespan: ", best_cost, " Total Cost: ", best_total_cost)

        if best_robot_path:
            best_plan_plot = plot_best_plan(env, best_obj_ins, goal_samples, best_robot_path, t)

        v_text = [round(best_cost,2), round(best_total_cost,2)]

        for name, ele in env.robots_map.items():
            ele.path = best_robot_path[name]
            ele.path_time = best_robot_path[name]['path_time']

        result_file.write("{},{},{},{},{},{},{},{},{}\n".format(t, init_task_cost, best_cost, best_total_cost, endtime,
                                                          roadmap_time, taskplan_time, probmake_time, motionplan_time))

        if t ==1:
            animation = Animation(env, best_obj_ins, goal_samples, best_robot_path, v_text)
            animation.save('figure/output_5s%d.mp4'%t, 2)
        # animation.show()

    result_file.close()
