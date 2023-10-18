import time as timer
from single_planner import compute_heuristics, a_star, get_sum_of_cost, get_sum_of_time, get_distance
from shapely.geometry import Point, LineString

class PrioritizedPlanningSolver(object):
    """
    A planner taht plans for each robot sequentially.
    {'robot0000': [[(-2.0, -4.0), (4.0, 3.5)], [(4.0, 3.5), 5.0, 'bolt'], [(4.0, 3.5), (-4.0, 3.5)], [(-4.0, 3.5), 5.0, 'bolt']], 'robot0001': [[(2.0, -4.0), (-4.0, 3.5)], [(-4.0, 3.5), 5.0, 'weld'], [(-4.0, 3.5), (2.0, -4.0)], [(2.0, -4.0), (4.0, 3.5)], [(4.0, 3.5), 5.0, 'weld']]}
    """

    def __init__(self, roadmap, samples, parsed_plan):
        """
        roadmap : [[1,2,3],[2,3,4],[0,2,3] ...]
        samples : [(0.3,2.1), (-2.5, 3.4), ... ]
        starts : [(x1, y1), (x2, y2), ...]
        goals : [(x1, y1), (x2, y2), ...]
        """
        self.starts = []
        self.goals = []
        self.parsed_plan = parsed_plan
        self.task_name = []
        self.goal_name = []

        self.robot_paths = dict()

        for name, ele in parsed_plan.items():
            self.robot_paths[name] = dict()
            temp_start = 0
            temp_goal = []
            temp_task = []
            temp_gname = []
            for way in ele:
                if not temp_start:
                    temp_start = way[0]
                if type(way[1]) == tuple:
                    temp_goal.append(way[1])
                else:
                    temp_goal.append(way[1])
                    temp_task.append(way[2])
                    temp_gname.append(way[3])

            self.task_name.append(temp_task)
            self.starts.append(temp_start)
            self.goals.append(temp_goal)
            self.goal_name.append(temp_gname)

        self.roadmap = roadmap
        self.samples = samples
        self.num_of_agents = len(self.starts)

        self.CPU_time = 0
        self.rr = 0.3

        # Heuristic gonna be dictionary or 0 - case of bolt/weld
        self.heuristics = []
        for i in range(self.num_of_agents):
            temp_heuristics = []
            for goal in self.goals[i]:
                if type(goal) == tuple:
                    temp_heuristics.append(compute_heuristics(roadmap, samples, goal))
                else:
                    temp_heuristics.append(0)
            self.heuristics.append(temp_heuristics)

    def parsed_path(self, path):
        time = []
        temp_path = []
        only_path = []
        cost_path = []
        for ele in path:
            if type(ele) == tuple:
                temp_path.append(ele)
            else:
                time.append(ele)
        if len(time) != len(temp_path):
            raise BaseException("len error!")
        temp_path.append(None)
        time.append(None)

        last_time = 0
        start_flag = False
        for i in range(len(temp_path)-1):
            if start_flag and temp_path[i] == temp_path[i+1]:
                pass
            elif not start_flag and temp_path[i] == temp_path[i+1]:
                start_flag = True
                last_time = time[i]
                only_path.append(temp_path[i])
                cost_path.append(temp_path[i])
            elif start_flag and temp_path[i] != temp_path[i+1]:
                start_flag = False
                cost_path.append([time[i] - last_time, 'wait'])
                last_time = 0
            else:
                last_time = 0
                cost_path.append(temp_path[i])
                only_path.append(temp_path[i])

        return cost_path, only_path

    def generate_path(self, path_cost, path_only):
        robot_id = 0
        for robot, ele in self.robot_paths.items():
            task_id = 0
            temp_path = []
            task_flag = False
            temp_task = []

            path1 = path_cost[robot_id]
            path2 = path_only[robot_id]
            for i in range(len(path1)):
                if type(path1[i]) == tuple and not task_flag:
                    temp_path.append(path1[i])
                elif type(path1[i]) == tuple and task_flag:
                    task_flag = False
                    temp_path.append(temp_task)
                    temp_task = []
                    temp_path.append(path1[i])
                elif type(path1[i]) == list and task_flag:
                    temp_task[1] += path1[i][0]
                    if path1[i][1] == 'task':
                        temp_task[2] += [(path1[i][0],self.task_name[robot_id][task_id], self.goal_name[robot_id][task_id])]
                        task_id += 1
                    elif path1[i][1] == 'wait':
                        temp_task[2] += [(path1[i][0], 2)]
                elif type(path1[i]) == list and not task_flag:
                    task_flag = True
                    temp_task.append(temp_path.pop())
                    temp_task.append(path1[i][0])
                    if path1[i][1] == 'task':
                        temp_task.append([(path1[i][0],self.task_name[robot_id][task_id], self.goal_name[robot_id][task_id])])
                        task_id += 1
                    elif path1[i][1] == 'wait':
                        temp_task.append([(path1[i][0], 2)])
            if temp_task and task_flag:
                temp_path.append(temp_task)
            ele['path_cost'] = temp_path
            ele['path_only'] = path2
            ele['path_time'] = get_sum_of_time(path1)

            robot_id += 1

    def find_solution(self):
        """
        find paths for all agent from their start locations to their goal locations.
        """
        start_time = timer.time()
        result = []
        result1 = []
        constraints = []

        for i in range(self.num_of_agents): # Find path for each agent
            temp_path_only = [self.starts[i]]
            temp_path_cost = [self.starts[i]]
            curr_time = 0
            # how many goal they have? -
            next_action = []
            for ele in self.goals[i]:
                if type(ele) == tuple:
                    next_action.append(0)
                else:
                    next_action.append(ele)
            if next_action[-1] > 0:
                next_action[-1] = -next_action[-1]
            next_action.reverse()
            next_action.pop()
            next_action.reverse()
            next_action.append(-1)

            for j, goal in enumerate(self.goals[i]):
                start = temp_path_only[-1]
                if type(goal) == tuple:
                    path = a_star(self.roadmap, self.samples, start, goal, self.heuristics[i][j],
                    			i, constraints, curr_time, next_action[j])
                    if path is None:
                        print("no path")
                        return None, -1, -1
                    else:
                        path1, path2 = self.parsed_path(path)
                        temp_path_cost = temp_path_cost + path1[1:]
                        temp_path_only = temp_path_only[:-1] + path2
                        curr_time += get_sum_of_time(path1)
                else:
                    curr_time += goal
                    temp_path_cost.append([goal, 'task'])

            result.append(temp_path_cost)
            result1.append(temp_path_only)

            curr_time = 0
            temp_const = dict()
            prev_loc = None
            for j in range(len(temp_path_cost)-1):
                way1 = temp_path_cost[j]
                way2 = temp_path_cost[j+1]
                if type(way1) != tuple:
                    way1 = prev_loc
                else:
                    prev_loc = way1

                if type(way2) == tuple:
                    next_time = curr_time + get_distance(way1,way2)
                    obs = LineString([way1, way2]).buffer(self.rr)
                else:
                    next_time = curr_time + way2[0]
                    obs = Point(way1).buffer(self.rr)
                temp_const[(curr_time, next_time)] = obs
                curr_time = next_time
            constraints.append(temp_const)

        self.generate_path(result, result1)
        print(self.robot_paths)
        makespan = 0
        total_cost = 0
        for name, ele in self.robot_paths.items():
            total_cost += ele['path_time']
            if makespan < ele['path_time']:
                makespan = ele['path_time']


        self.CPU_time = timer.time() - start_time
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(makespan))
        return self.robot_paths, makespan, total_cost
