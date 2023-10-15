#!/usr/bin/python3
import argparse
import time
from pathlib import Path
import subprocess
import matplotlib.pyplot as plt
from environment import Environment
from prm import PRMPlanning
from pddlgenerator import PDDLProblemGenerator, read_domain
from voronoi import VoronoiPlanning
from visualize import *
# from collision_vor import *
import os
import glob




if __name__ == '__main__':
    """
    Argument Management
    Map: Yaml File (Robot Config, Goals, Env (obstacle))
    Domain PDDL: Domain File (Weld-Bolt)
    Problem PDDL: File name generated from module
    Solver: CBTAMP, Prioritized, Taskonly
    """
    parser = argparse.ArgumentParser(description='Runs Task and Motion Planning Module')
    parser.add_argument('-m', '--map', type=str, default = None,
                        help = 'The name of an instance file')
    parser.add_argument('-d', '--domain', type=str, default = None,
                        help = 'The name of input domain file')
    parser.add_argument('-p', '--problem', type=str, default = 'example_problem.pddl',
                        help = 'The name of output problem file')
    parser.add_argument('-s', '--solver', type=str, default = 'CBTAMP',
                        help = 'The name of solver')

    parser.add_argument('-pl', '--plot', action='store_true', default = False,
                        help = 'Use batch output instead of plot')
    parser.add_argument('-v', '--vor', action='store_true', default = False,
                        help = 'Use Voronoi Samples for Planning')
    parser.add_argument('-t', '--trial', type=int, default = 1,
                        help = 'Experiment Number')
    args = parser.parse_args()

    dFile = args.domain
    mFile = args.map
    pFile = args.problem
    solver = args.solver

    plot = args.plot
    voronoi = args.vor
    trial = args.trial

    additional_domain = True

    # Domain, Problem File Setting
    d_name = read_domain(dFile)
    if additional_domain:
        late_dFile = 'pddl_domain/late_cbtamp.pddl'
        const_dFile = 'pddl_domain/const_cbtamp.pddl'

    # Visualization Setting
    env_plot = None
    if voronoi:
        voronoi_plot = None
    prm_plot = None
    plan_plot = None
    collision_plot = None
    obj_plot = None

    # CSV File Setting
    result_file = open("results.csv", "w", buffering=1)

    figs = glob.glob('figure/*.png')
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
        colcheck_time = 0
        probmake_time = 0

        # Load Environment from yaml File
        # and Visualize
        start_time = time.time()
        test_time = time.time()
        env = Environment(mFile)
        if plot:
            env_plot = plot_environment(env, t)

        # Voronoi Sampling
        v_sample = []
        if voronoi:
            vor = VoronoiPlanning(env)
            v_sample = vor.voronoi_sampling()

        """
        PRM Generate Part from map file
        Samples [it can be used for waypoint in problem file]
        Roadmap [it can be used for edge in problem file]
        Goal_Sample [it must be used for waypoint and job located in problem file]
        """
        prm = PRMPlanning(env, v_sample, solver)
        init_samples, init_roadmap, goal_samples = prm.ConstructPhase(goals_map = env.goals_map)

        roadmap_time = time.time() - test_time


        vor_way = []
        if voronoi:
            vor_way = vor.pick_for_task(env.robots_map, goal_samples)
            prm.vor_ways = vor_way

        # Visualizing PRM
        if plot:
            if voronoi:
                voronoi_plot = plot_voronoi(env, vor_way, goal_samples, t)
            prm_plot = plot_prm(env, init_samples, init_roadmap, goal_samples, t)

        """
        Task Planning Domain
        """
        count = 0

        test_time = time.time()
        makeprob = PDDLProblemGenerator(env, prm, vor_way, d_name, pFile)
        if solver == 'Taskonly':
            makeprob.task_env_instance()
        else:
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
        init_plan_dispatch, init_task_cost, init_cal_time = makeprob.parse_pddl_plan(planFile)
        init_robot_path, init_total_cost, init_motion_cost = makeprob.generate_path(init_plan_dispatch)
        elapsed_time = time.time() - test_time
        motion_time += elapsed_time

        if plot:
            early_plan_plot = plot_plan(env, makeprob, goal_samples, init_robot_path, count, 0)

        test_time = time.time()
        col_env = []
        # Initial Problem doesn't need early or late check
        early_collision, col_env = collision_check(init_robot_path, env.robots_map, prm, col_env)
        elapsed_time = time.time() - test_time
        collision_time += elapsed_time

        if not early_collision:
            best_cost = init_motion_cost
            best_robot_path = init_robot_path
            best_total_cost = init_total_cost
            best_index = [count, 0]
            best_obj_ins = copy.deepcopy(makeprob.obj_ins)

        next_collisions = early_collision
        exit_flag = False

        while next_collisions:
            count += 1
            # if count > 3 and exit_flag:
            #     break
            if exit_flag:
                break
            if count > 10:
                break

            if plot:
                col_plot = plot_collision(env, prm, next_collisions, goal_samples, count)
                col_update_plot = plot_collision_update(env, prm, col_env, goal_samples, count)
            next_collisions = []

            test_time = time.time()
            makeprob.env_update(col_env, prm.vor_ways)
            elapsed_time = time.time() - test_time
            env_update_time += elapsed_time
            if debug_mode:
                print("env_update_time: ", elapsed_time)

            test_time = time.time()
            pName = makeprob.generate_problem(distance, count)
            elapsed_time = time.time() - test_time
            pddl_make_time += elapsed_time
            if debug_mode:
                print("pddl_make_time: ", elapsed_time)

            if plot:
                obj_plot = plot_obj_check(env, makeprob, goal_samples, count)

            test_time = time.time()
            early_plan = 'pddl_prob_plan/%d_e_plan%d.pddl'%(t+1, count)
            late_plan = 'pddl_prob_plan/%d_l_plan%d.pddl'%(t+1, count)
            const_plan = 'pddl_prob_plan/%d_c_plan%d.pddl'%(t+1, count)

            command = './planners/optic-cplex-wrapper ' + early_dFile + ' ' + pName + ' ' + early_plan
            popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            (stdoutdata, stderrdata) = popen.communicate()

            command = './planners/optic-cplex-wrapper ' + late_dFile + ' ' + pName + ' ' + late_plan
            popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            (stdoutdata, stderrdata) = popen.communicate()

            # command = './planners/optic-cplex-wrapper ' + const_dFile + ' ' + pName + ' ' + const_plan
            # popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            # (stdoutdata, stderrdata) = popen.communicate()

            early_f = Path(early_plan)
            if not early_f.is_file():
                print("Use -n -c -S command for early_plan")
                command = './planners/optic-cplex-wrapper_c ' + early_dFile + ' ' + pName + ' ' + early_plan
                popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
                (stdoutdata, stderrdata) = popen.communicate()
            late_f = Path(late_plan)
            if not late_f.is_file():
                print("Use -n -c -S command for late_plan")
                command = './planners/optic-cplex-wrapper_c ' + late_dFile + ' ' + pName + ' ' + late_plan
                popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
                (stdoutdata, stderrdata) = popen.communicate()
            const_f = Path(const_plan)
            # if not const_f.is_file():
            #     print("Use -n -c -S command for const_plan")
            #     command = './planners/optic-cplex-wrapper_c ' + const_dFile + ' ' + pName + ' ' + const_plan
            #     popen = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            #     (stdoutdata, stderrdata) = popen.communicate()
            #     const_f = Path(const_plan)

            elapsed_time = time.time() - test_time
            optic_time += elapsed_time

            if debug_mode:
                print("--------------------------------------------------------")
                print("Count: ", count)

            test_time = time.time()

            early_robot_path = []
            if early_f.is_file():
                early_plan_dispatch, early_task_cost, early_cal_time = makeprob.parse_pddl_plan(early_plan)
                early_robot_path, early_total_cost, early_motion_cost = makeprob.generate_path(early_plan_dispatch)
                if debug_mode:
                    print("Early Plan")
                    print("Task Plan Cost: ", early_task_cost, " Calculation Time: ", early_cal_time, " Motion Planner Path Total Cost: ",
                        early_total_cost, " Motion Planner Path Max Cost: ", early_motion_cost)

            late_robot_path = []
            if late_f.is_file():
                late_plan_dispatch, late_task_cost, late_cal_time = makeprob.parse_pddl_plan(late_plan)
                late_robot_path, late_total_cost, late_motion_cost = makeprob.generate_path(late_plan_dispatch)
                if debug_mode:
                    print("Late Plan")
                    print("Task Plan Cost: ", late_task_cost, " Calculation Time: ", late_cal_time, " Motion Planner Path Total Cost: ",
                        late_total_cost, " Motion Planner Path Max Cost: ", late_motion_cost)

            const_robot_path = []
            # if const_f.is_file():
            #     const_plan_dispatch, const_task_cost, const_cal_time = makeprob.parse_pddl_plan(const_plan)
            #     const_robot_path, const_total_cost, const_motion_cost = makeprob.generate_path(const_plan_dispatch)
            #     if debug_mode:
            #         print("Const Plan")
            #         print("Task Plan Cost: ", const_task_cost, " Calculation Time: ", const_cal_time, " Motion Planner Path Total Cost: ",
            #                 const_total_cost, " Motion Planner Path Max Cost: ", const_motion_cost)

            elapsed_time = time.time() - test_time
            motion_time += elapsed_time

            if not late_robot_path and not early_robot_path:
                print("Failed to plan every domain.")
                break

            if plot:
                early_plan_plot = plot_plan(env, makeprob, goal_samples, early_robot_path, count, 1)
                late_plan_plot = plot_plan(env, makeprob, goal_samples, late_robot_path, count, 2)
                if const_robot_path:
                    const_plan_plot = plot_plan(env, makeprob, goal_samples, const_robot_path, count, 3)

            test_time = time.time()
            early_collision = []
            late_collision = []
            const_collision = []
            if early_robot_path:
                early_collision, col_env_e = collision_check(early_robot_path, env.robots_map, prm, col_env)
            if late_robot_path:
                if early_collision:
                    late_collision, col_env_l = collision_check(late_robot_path, env.robots_map, prm, -1)
                else:
                    late_collision, col_env_l = collision_check(late_robot_path, env.robots_map, prm, col_env)
            if const_robot_path:
                const_collision, col_env_c = collision_check(const_robot_path, env.robots_map, prm, -1)
            elapsed_time = time.time() - test_time
            collision_time += elapsed_time

            # if const_robot_path:
            #     if not const_collision:
            #         print("Const Plan Success")
            #         if const_motion_cost < best_cost:
            #             best_cost = const_motion_cost
            #             best_robot_path = const_robot_path
            #             best_total_cost = const_total_cost
            #             best_index = [count, 2]
            #             best_obj_ins = copy.deepcopy(makeprob.obj_ins)
            #             best_col_env = copy.deepcopy(col_env)

            if late_robot_path:
                if not late_collision:
                    print("Late Plan Success")
                    exit_flag = True
                    if late_motion_cost < best_cost:
                        best_cost = late_motion_cost
                        best_robot_path = late_robot_path
                        best_total_cost = late_total_cost
                        best_index = [count, 1]
                        best_obj_ins = copy.deepcopy(makeprob.obj_ins)
                        best_col_env = copy.deepcopy(col_env)
                else:
                    next_collisions = late_collision
                    col_env = col_env_l

            if early_robot_path:
                if not early_collision:
                    print("Early Plan Success")
                    exit_flag = True
                    if early_motion_cost < best_cost:
                        best_cost = early_motion_cost
                        best_robot_path = early_robot_path
                        best_total_cost = early_total_cost
                        best_index = [count, 0]
                        best_obj_ins = copy.deepcopy(makeprob.obj_ins)
                        best_col_env = copy.deepcopy(col_env)
                else:
                    next_collisions = early_collision
                    col_env = col_env_e
        ##############################################################################################

        if not best_robot_path:
            print("Mission Failed")
            col_update_plot = plot_collision_update(env, prm, col_env, goal_samples, t+1)
            obj_plot = plot_obj_check(env, makeprob, goal_samples, t+1)
            continue

        endtime = time.time() - start_time
        print("Mission Complete")

        print("ENV TIME: ", env_ins_time)
        print("CONSTRUCT TIME: ", construct_time)
        print("TASK PLANNER TIME: ", optic_time)
        print("MAKE PDDL TIME: ", pddl_make_time)
        print("ENV UPDATE TIME: ", env_update_time)
        print("MOTION TIME: ", motion_time)
        print("COLLISION TIME: ", collision_time)

        r_time = env_ins_time + construct_time
        t_time = optic_time + pddl_make_time
        m_time = env_update_time + motion_time + collision_time
        print("ROADMAP MODULE: ", env_ins_time + construct_time)
        print("TASK MODULE: ", optic_time + pddl_make_time)
        print("MOTION MODULE: ", env_update_time + motion_time)
        print("COLLISION MODULE: ", collision_time)

        print("Total Time: ", endtime)
        if best_index[1] == 0:
            print(best_index[0], "th Early Plan Selected!, Makespan: ", best_cost, " Total Cost: ", best_total_cost)
        elif best_index[1] == 1:
            print(best_index[0], "th Late Plan Selected!, Makespan: ", best_cost, " Total Cost: ", best_total_cost)
        elif best_index[1] == 2:
            print(best_index[0], "th Constraint Plan Selected!, Makespan: ", best_cost, " Total Cost: ", best_total_cost)

        # if best_col_env:
        #     col_update_plot = plot_collision_update(env, prm, best_col_env, goal_samples, t+1)
        # if best_obj_ins:
        #     obj_plot = plot_obj_final(env, best_obj_ins, goal_samples, t+1)
        if best_robot_path:
            best_plan_plot = plot_best_plan(env, best_obj_ins, goal_samples, best_robot_path, t+1)

        # if plot:
        #     plt.show()

        v_text = [round(best_cost,2), round(best_total_cost,2)]

        for name, ele in env.robots_map.items():
            ele.path = best_robot_path[name]
            ele.path_time = best_robot_path[name]['path_time']

        result_file.write("{},{},{},{},{},{},{}\n".format(t+1, best_cost, best_total_cost, endtime, r_time, t_time, m_time))

        animation = Animation(env, best_obj_ins, goal_samples, best_robot_path, v_text)
        animation.save('figure/output_5s%d.mp4'%(t+1), 2)
        # animation.show()
    result_file.close()
