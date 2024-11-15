from pathlib import Path
from shapely.geometry import Point, LineString
import copy
from utils import get_sum_of_cost, KDTree, get_distance

from visualize import *

class PDDLProblemGenerator:
    """
    Problem Generator Class
    """
    def __init__(self, env, prm, vor_way, domain, pFile):
        self.env = env
        self.robots = env.robots_map
        self.goals = env.goals_map
        self.prm = prm
        self.samples = prm.samples
        self.roadmaps = prm.roadmaps
        self.task_samples = prm.max_samples
        self.task_roadmap = prm.max_roadmap
        self.task_rr = prm.max_rr
        self.vor_way = vor_way

        self.pFilename = pFile
        self.pFile = None
        self.domain = domain

        self.obj_ins = dict()
        self.basic_way_num = 0

        # self.velocity = 1.0

    class Instance:
        def __init__(self, ind, loc, instype):
            # Common Usage for every Instance
            # ind = 'waypoint', 'robot', 'job', 'token', 'entry'
            # loc = (x,y)
            # instype = Each instance type. Ex) Robot = 'wrobot', 'srobot'
            self.ind = ind
            self.loc = loc
            self.instype = instype

            # Waypoint Var
            self.connected = []
            self.distance = []
            self.path = []
            self.vertex = True  # If vertex is False there is already occupied with one robot
            self.collision_connect = []
            self.reserved = False
            self.token_ind = None

            self.sample_ind = None
            self.connect_ind = []

            # Job Var
            self.located = []
            self.order = []

            # Robot Var
            self.at = []
            self.action_duration = dict()
            self.velocity = 0

            # Token Var
            self.token_name = ''
            self.token_wp = []
            self.token_samples = []

    def instance_init(self):
        self.obj_ins = dict()

    def task_env_instance(self):
        # Robot
        wpind = 0
        for name, robot_map in self.robots.items():
            pos = robot_map.pos
            waypoint = self.Instance('wp'+str(wpind), pos, 'normal_way')
            waypoint.vertex = False
            waypoint.sample_ind = self.task_samples.index(pos)
            waypoint.connect_ind = self.task_roadmap[waypoint.sample_ind]
            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]

            robot = None
            robot = self.Instance(name, pos, 'robot')
            for i in range(len(robot_map.service)):
                robot.action_duration[robot_map.service[i]] = robot_map.action_cost[i]
            robot.at.append('wp'+str(wpind))
            robot.velocity = robot_map.max_speed
            self.obj_ins['robot'] = self.obj_ins.get('robot', []) + [robot]
            wpind += 1

        # Goal
        temp_task = dict()
        for name, goal_map in self.goals.items():
            job = self.Instance(name, goal_map['samples'], 'job')
            for i in range(len(goal_map['service'])):
                temp_task[(goal_map['service'][i], goal_map['service_type'][i])] = temp_task.get((goal_map['service'][i], goal_map['service_type'][i]), []) + [name]
            for sample in goal_map['samples']:
                waypoint = self.Instance('wp'+str(wpind), sample, 'normal_way')
                waypoint.sample_ind = self.task_samples.index(sample)
                waypoint.connect_ind = self.task_roadmap[waypoint.sample_ind]
                job.located.append('wp'+str(wpind))
                wpind += 1
                self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]
            for ele in goal_map['order']:
                job.order.append(ele)
            self.obj_ins['job'] = self.obj_ins.get('job', []) + [job]

        check_loc = []
        for ele in self.obj_ins['waypoint']:
            check_loc.append(ele.loc)

        # Task
        for name, task_map in temp_task.items():
            name1, name2 = name
            task = self.Instance(name1, task_map, name2)
            self.obj_ins['task'] = self.obj_ins.get('task', []) + [task]
        self.basic_way_num = len(self.obj_ins['waypoint'])

        # waypoint
        for i, way in enumerate(self.task_samples):
            if way in check_loc:
                continue
            waypoint = self.Instance('wp'+str(wpind), way, 'normal_way')
            waypoint.sample_ind = i
            waypoint.connect_ind = self.task_roadmap[waypoint.sample_ind]

            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]
            wpind += 1

        # Calculate Distance of goal, start waypoints
        way_list = self.obj_ins['waypoint']

        for i in range(len(way_list)-1):
            for j in range(i+1, len(way_list)):
                start_pos = way_list[i].loc
                goal_pos = way_list[j].loc
                goal_ind = way_list[j].sample_ind
                if goal_ind in way_list[i].connect_ind:
                    temp_dist = round(get_distance(start_pos, goal_pos), 3)
                    if way_list[j].vertex:
                        way_list[i].distance.append(temp_dist)
                        way_list[i].connected.append(way_list[j].ind)
                        way_list[i].path.append([start_pos, goal_pos])
                    if way_list[i].vertex:
                        way_list[j].distance.append(temp_dist)
                        way_list[j].connected.append(way_list[i].ind)
                        way_list[j].path.append([goal_pos, start_pos])

    def env_instance(self):
        # Robot
        wpind = 0
        for name, robot_map in self.robots.items():
            pos = robot_map.pos
            waypoint = self.Instance('wp'+str(wpind), pos, 'normal_way')
            waypoint.vertex = False
            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]

            robot = None
            robot = self.Instance(name, pos, 'robot')
            for i in range(len(robot_map.service)):
                robot.action_duration[robot_map.service[i]] = robot_map.action_cost[i]
            robot.at.append('wp'+str(wpind))
            robot.velocity = robot_map.max_speed
            self.obj_ins['robot'] = self.obj_ins.get('robot', []) + [robot]
            wpind += 1

        # Goal
        temp_task = dict()
        for name, goal_map in self.goals.items():
            job = self.Instance(name, goal_map['samples'], 'job')
            for i in range(len(goal_map['service'])):
                temp_task[(goal_map['service'][i], goal_map['service_type'][i])] = temp_task.get((goal_map['service'][i], goal_map['service_type'][i]), []) + [name]
            for sample in goal_map['samples']:
                new_flag = True
                for i in self.obj_ins['waypoint']:
                    if sample == i.loc:
                        job.located.append(i.ind)
                        new_flag = False
                        break
                if new_flag:
                    waypoint = self.Instance('wp'+str(wpind), sample, 'normal_way')
                    job.located.append('wp'+str(wpind))
                    wpind += 1
                    self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]
            for ele in goal_map['order']:
                job.order.append(ele)
            self.obj_ins['job'] = self.obj_ins.get('job', []) + [job]

        # Task
        for name, task_map in temp_task.items():
            name1, name2 = name
            task = self.Instance(name1, task_map, name2)
            self.obj_ins['task'] = self.obj_ins.get('task', []) + [task]
        self.basic_way_num = len(self.obj_ins['waypoint'])

        # Voronoi
        for v_way in self.vor_way:
            waypoint = self.Instance('wp'+str(wpind), v_way, 'normal_way')
            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]
            wpind += 1

        # Calculate Distance of goal, start waypoints
        way_list = self.obj_ins['waypoint']

        temp_samples = []
        remove_samples = []
        for way in way_list:
            way_buffer = Point(way.loc).buffer(self.task_rr)
            for w in self.task_samples:
                w_point = Point(w).buffer(self.task_rr)
                if way_buffer.intersects(w_point):
                    remove_samples.append(w)
        for w in self.task_samples:
            if not w in remove_samples:
                temp_samples.append(w)
        temp_samples = list(set(temp_samples))
        temp_skdtree = KDTree(temp_samples)
        temp_roadmap = []
        cd = self.prm.casting_dist[self.task_rr]
        nsample = len(temp_samples)
        knn = min(int(len(self.task_samples)/40), 15)
        for (ii, temp_point) in zip(range(nsample), temp_samples):
            index, dists = temp_skdtree.search(temp_point, k=nsample)
            edge_id = []
            for jj in range(1, len(index)):
                npoint = temp_samples[index[jj]]
                if self.prm.isEdgeCollisionFree(temp_point, npoint, self.task_rr) and get_distance(temp_point, npoint) < cd:
                    edge_id.append(index[jj])
                if len(edge_id) >= knn:
                    break
            temp_roadmap.append(edge_id)

        for i in range(len(way_list)-1):
            for j in range(i+1, len(way_list)):
                start_pos = way_list[i].loc
                goal_pos = way_list[j].loc
                temp_path1, temp_dist1, temp_path2, temp_dist2 = self.prm.QuerywithRoadmap(start_pos, goal_pos,
                                                        temp_samples, temp_roadmap, temp_skdtree, self.task_rr)
                if temp_dist1 != -1:
                    temp_path, temp_dist = self.prm.QueryTaskPlan(start_pos, goal_pos)
                    way_list[i].distance.append(temp_dist)
                    way_list[i].connected.append(way_list[j].ind)
                    way_list[i].path.append(temp_path)
                if temp_dist2 != -1:
                    temp_path, temp_dist = self.prm.QueryTaskPlan(goal_pos, start_pos)
                    way_list[j].distance.append(temp_dist)
                    way_list[j].connected.append(way_list[i].ind)
                    way_list[j].path.append(temp_path)

    def generate_header(self):
        """
        Make Header for Problem PDDL File
        Header includes domain definition and objects
        """
        # Domain Definition
        self.pFile.write('(define (problem ' + self.pFilename.replace('.pddl','') + ')\n')
        self.pFile.write('(:domain ' + self.domain + ')\n')
        self.pFile.write('(:objects\n')

        # Object Adding
        # They are ordered by instype not ins (for example weld_robot, bolt_robot not robot)
        for name, obj in self.obj_ins.items():
            temp_dict = dict()
            for ele in obj:
                temp_dict[ele.instype] = temp_dict.get(ele.instype, []) + [ele.ind]
            for obj_type, ind in temp_dict.items():
                if obj_type == 'token':
                    break
                self.pFile.write('        ')
                counter = 1
                for ele in ind:
                    self.pFile.write(ele + ' ')
                    if counter == len(ind):
                        self.pFile.write('- ' + obj_type + '\n')
                    elif counter % 5 == 0:
                        self.pFile.write('- ' + obj_type + '\n')
                        self.pFile.write('        ')
                    counter += 1
        self.pFile.write(')\n')

    def generate_initial(self):
        """
        Make Initial State for Problem File
        Waypoint need 1. vertex free, 2. edge free, 3. connected, 4. distance
        Robot need 1. at, 2. velocity
        Job need 1. located, 2. service, 3. work free (there are many waypoints to work on job, so we limited it only at once one job)
        """
        self.pFile.write('(:init\n')

        for name, obj in self.obj_ins.items():
            for ele in obj:
                if name == 'waypoint':
                    if ele.vertex:
                        self.pFile.write('        ')
                        self.pFile.write('(vertex_free ' + ele.ind + ')\n')
                    # if ele.reserved:
                    #     self.pFile.write('        ')
                    #     self.pFile.write('(reserved ' + ele.ind + ')\n')
                    for i in range(len(ele.connected)):
                        self.pFile.write('        ')
                        self.pFile.write('(connected ' + ele.ind + ' ' + ele.connected[i] + ')\n')
                        self.pFile.write('        ')
                        self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                        self.pFile.write(str(ele.distance[i]) + ')\n')
                elif ele.instype.find('robot') > -1:
                    #print(ele.instype)
                    self.pFile.write('        ')
                    self.pFile.write('(at ' + ele.ind + ' ' + ele.at[0] + ')\n')
                    # self.pFile.write('        ')
                    # self.pFile.write('(robot_free ' + ele.ind + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(= (velocity ' + ele.ind + ') ' + str(ele.velocity) + ')\n')
                    for task_name, action_cost in ele.action_duration.items():
                        self.pFile.write('        ')
                        self.pFile.write('(= (task_duration ' + ele.ind + ' ' + task_name + ') ' + str(action_cost) + ')\n')
                        self.pFile.write('        ')
                        self.pFile.write('(can_perform ' + ele.ind + ' ' + task_name + ')\n')

                elif ele.instype == 'job':
                    for wp in ele.located:
                        self.pFile.write('        ')
                        self.pFile.write('(located ' + ele.ind + ' ' + wp + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(work_free ' + ele.ind + ')\n')
                    for order in ele.order:
                        self.pFile.write('        ')
                        self.pFile.write('(before ' + ele.ind + ' ' + order[0] + ' ' + order[1] + ')\n')
                elif name == 'task':
                    goal_locs = ele.loc
                    for goal_loc in goal_locs:
                        self.pFile.write('        ')
                        self.pFile.write('(isJobOfType ' + goal_loc + ' ' + ele.ind + ')\n')

                self.pFile.write('\n')

        self.pFile.write(')\n')

    def generate_goals(self):
        """
        Generate Goal for Problem File
        now only service needed for it.
        """
        self.pFile.write('(:goal (and\n')
        for task in self.obj_ins['task']:
            for ele in task.loc:
                self.pFile.write('        ')
                self.pFile.write('(finish ' + ele + ' ' + task.ind + ')\n')
        self.pFile.write('        )\n')
        self.pFile.write(')\n')

    def generate_metric(self):
        """
        Metric func for minimizing something
        """
        self.pFile.write('(:metric minimize (total-time))\n')

    def generate_problem(self, t, count = 0):
        self.pFilename.replace('.pddl','')
        pName = 'pddl_prob_plan/%d_'%t + self.pFilename.replace('.p','%d.p'%count)
        self.pFile = open(pName, "w", buffering=1)
        self.generate_header()
        self.generate_initial()
        self.generate_goals()
        self.generate_metric()
        self.pFile.write(')\n')
        self.pFile.close()
        return pName

    def parse_pddl_plan(self, planFile):
        """
        Plan Reading and Parse Function. They only parse the plan, no refine
        planfile : Plan from optic wrapper
        @Return pair of plan dispatch (robot-goals)
        @Retrun Path Data Structure like
        [point-Tuple(float, float), dispatch time -float, action_time - float]
        """
        f = Path(planFile)
        if not f.is_file():
            raise BaseException(planFile + " does not exists.")
        f = open(planFile, 'r')
        lines = f.readlines()
        path = dict() # For robot inital path
        way_dict = dict()
        plan_list = dict() # For Prioritized Planning
        cost = 0
        for r in self.obj_ins['robot']:
            path[r.ind] = []
            plan_list[r.ind] = []
        for w in self.obj_ins['waypoint']:
            way_dict[w.ind] = dict()
            way_dict[w.ind]['loc'] = w.loc
            for i in range(len(w.connected)):
                way_dict[w.ind][w.connected[i]] = w.path[i]

        for line in lines:
            line = line.strip()
            if 'metric' in line:
                ind = line.find('metric')
                line = line.replace('metric ', '')
                cost = float(line[ind:])
            if 'Time' in line:
                ind = line.find('Time')
                line = line.replace('Time ','')
                cal_time = float(line[ind:])
            if 'Cost:' in line:
                ind = line.find('Cost:')
                line = line.replace('Cost: ','')
                cost = float(line[ind:])
            if '_navigate' in line:
                for r in path:
                    if r in line:
                        # Dispatch Time
                        time_ind = line.find(':')
                        dispatch_t = float(line[:time_ind])
                        # Waypoint 1
                        wp1_ind = line.find('wp')
                        sub1_line = line[wp1_ind:]
                        space_ind = sub1_line.find(' ')
                        wp1 = sub1_line[:space_ind]
                        # Waypoint 2
                        sub2_line = line[wp1_ind+2:]
                        wp2_ind = sub2_line.find('wp')
                        sub2_line = sub2_line[wp2_ind:]
                        blanket_ind = sub2_line.find(')')
                        space_ind = sub2_line.find(' ')
                        wp2 = sub2_line[:min(space_ind, blanket_ind)]
                        # Path
                        wp1_temp = None
                        wp2_temp = None
                        if wp1 in way_dict:
                            wp1_temp = way_dict[wp1]['loc']
                        if wp2 in way_dict:
                            wp2_temp = way_dict[wp2]['loc']
                        local_path = None
                        if wp1 in way_dict:
                            if wp2 in way_dict[wp1]:
                                local_path = copy.deepcopy(way_dict[wp1][wp2])
                        # Action duration
                        bf_ind = line.find('[')
                        be_ind = line.find(']')
                        action_d = float(line[bf_ind+1:be_ind])
                        path[r].append([local_path, dispatch_t, action_d])
                        plan_list[r].append([wp1_temp, wp2_temp])
            if 'do_task_single' in line:
                for r in path:
                    if r in line:
                        # Dispatch Time
                        time_ind = line.find(':')
                        dispatch_t = float(line[:time_ind])
                        # Waypoint 1
                        wp1_ind = line.find('wp')
                        sub1_line = line[wp1_ind:]
                        space_ind = sub1_line.find(' ')
                        wp1 = sub1_line[:space_ind]
                        if wp1 in way_dict:
                            wp1 = way_dict[wp1]['loc']
                        goal_ind = line.find('goal')
                        b_ind = line.find(')')
                        line2 = line[goal_ind:b_ind]
                        s_ind = line2.find(' ')
                        goal_name = line2[:s_ind]
                        task_name = line2[s_ind+1:]

                        bf_ind = line.find('[')
                        be_ind = line.find(']')
                        action_d = float(line[bf_ind+1:be_ind])
                        path[r].append([[wp1], dispatch_t, action_d, task_name, goal_name])
                        plan_list[r].append([wp1, action_d, task_name, goal_name])
            if 'do_task_double' in line:
                for r in path:
                    if r in line:
                        # Dispatch Time
                        time_ind = line.find(':')
                        dispatch_t = float(line[:time_ind])
                        # Waypoint 1
                        robot_ind = line.find(r)
                        robot_line = line[robot_ind:]
                        wp_ind = robot_line.find('wp')
                        wp_line = robot_line[wp_ind:]
                        space_ind = wp_line.find(' ')
                        wp1 = wp_line[:space_ind]
                        if wp1 in way_dict:
                            wp1 = way_dict[wp1]['loc']

                        # for goal
                        goal_ind = line.find('goal')
                        b_ind = line.find(')')
                        line2 = line[goal_ind:b_ind]
                        s_ind = line2.find(' ')
                        goal_name = line2[:s_ind]
                        task_name = line2[s_ind+1:]

                        bf_ind = line.find('[')
                        be_ind = line.find(']')
                        action_d = float(line[bf_ind+1:be_ind])
                        path[r].append([[wp1], dispatch_t, action_d, task_name, goal_name])
                        plan_list[r].append([wp1, action_d, task_name, goal_name])
        f.close()

        return path, cost, cal_time, plan_list

    def generate_path(self, parsed_path):
        """
        parsed_path to robot_path
        @Retrun Path Data Structure like
        """
        robot_path = dict()
        total_cost = 0
        max_cost = 0
        for name, ele in parsed_path.items():
            combined_path = dict()
            path_cost = []
            path_only = []
            action_time = 0.0

            if len(ele) == 0:
                pass

            if len(ele) == 1:
                if len(ele[0][0]) == 1:
                    path_cost = [ele[0][0][0], ele[0][2], [(ele[0][2],ele[0][3],ele[0][4])]]
                    path_only = ele[0][0]
                    action_time = ele[0][2]
                else:
                    path_cost = ele[0][0]
                    path_only = ele[0][0]
            else:
                path_only = [self.robots[name].pos]
                path_cost = [self.robots[name].pos]
                for i in range(len(ele)-1):
                    curr_plan = ele[i][0]
                    curr_time = ele[i][1]
                    curr_actioncost = ele[i][2]
                    next_time = ele[i+1][1]
                    if path_only[-1] != curr_plan[0]:
                        raise Exception("Service Location Error!")
                    if len(curr_plan) == 1:
                        action_time += curr_actioncost
                        if type(path_cost[-1]) == list:
                            path_cost[-1][1] += curr_actioncost
                            path_cost[-1][2] += [(ele[i][2],ele[i][3],ele[i][4])]
                        else:
                            path_cost[-1] = [path_cost[-1], curr_actioncost, [(ele[i][2],ele[i][3],ele[i][4])]]
                    else:
                        temp_path = copy.deepcopy(curr_plan)
                        path_only += curr_plan[1:]
                        path_cost += temp_path[1:]
                    waiting_time = round(next_time - (curr_time + curr_actioncost), 3)
                    if waiting_time > 0.01:
                        action_time += waiting_time
                        if type(path_cost[-1]) == list:
                            path_cost[-1][1] += waiting_time
                            path_cost[-1][2] += [(waiting_time, 2)]
                        else:
                            path_cost[-1] = [path_cost[-1], waiting_time, [(waiting_time, 2)]]
                last_plan = ele[-1][0]
                last_actioncost = ele[-1][2]
                if path_only[-1] != last_plan[0]:
                    raise Exception("Service Location Error!")
                if len(last_plan) == 1:
                    action_time += last_actioncost
                    if type(path_cost[-1]) == list:
                        path_cost[-1][1] += last_actioncost
                        path_cost[-1][2] += [(ele[-1][2],ele[-1][3],ele[-1][4])]
                    else:
                        path_cost[-1] = [path_cost[-1], last_actioncost, [(ele[-1][2],ele[-1][3],ele[-1][4])]]
                else:
                    temp_path = copy.deepcopy(last_plan)
                    path_only += last_plan[1:]
                    path_cost += temp_path[1:]

            combined_path['path_cost'] = path_cost
            combined_path['path_only'] = path_only
            combined_path['path_time'] = round(action_time + get_sum_of_cost(path_only), 3)
            max_cost = max(max_cost, combined_path['path_time'])
            total_cost += combined_path['path_time']
            robot_path[name] = combined_path

        for name, ele in self.robots.items():
            if name in robot_path:
                if robot_path[name]['path_time'] != max_cost:
                    time_remain = round(max_cost - robot_path[name]['path_time'], 3)
                    if type(robot_path[name]['path_cost'][-1]) == list:
                        robot_path[name]['path_cost'][-1][1] += time_remain
                        robot_path[name]['path_time'] = max_cost
                        robot_path[name]['path_cost'][-1][2] += [(time_remain, 2)]
                    else:
                        robot_path[name]['path_cost'][-1] = [robot_path[name]['path_cost'][-1], time_remain, [(time_remain, 2)]]
                        robot_path[name]['path_time'] = max_cost
            else:
                path_only = [self.robots[name].pos]
                path_cost = [[self.robots[name].pos, max_cost, [(max_cost, 2)]]]
                robot_path[name] = dict()
                robot_path[name]['path_only'] = path_only
                robot_path[name]['path_cost'] = path_cost
                robot_path[name]['path_time'] = max_cost

        return robot_path, total_cost, max_cost

def read_domain(dFile):
    """
    Domain Reading Func
    dFile: Domain File Path
    @Return domain_name(string), domain type(int)
    """
    f = Path(dFile)
    if not f.is_file():
        raise BaseException(dFile + " does not exists.")
    f = open(dFile, 'r')
    lines = f.readlines()
    domain_name = ''

    for line in lines:
        line = line.strip()
        # if we need more info from domain file, remove break and read more lines here.
        if line.startswith('(define'):
            start = line.find('domain ')
            line = line.replace('domain ','')
            end = line.find(')')
            domain_name = line[start:end]
            break
    f.close()

    return domain_name
