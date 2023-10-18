import heapq
import math
from copy import deepcopy
from shapely.geometry import Point, LineString

res = 1

def get_sum_of_cost(path):
    rst = 0
    start = path[0]
    for i in range(1,len(path)):
        rst += get_distance(start,path[i])
        start = path[i]
    return rst

def get_sum_of_time(path):
    rst = 0
    temp_path = []
    for ele in path:
        if type(ele) == tuple:
            temp_path.append(ele)
        elif type(ele) == list:
            rst += ele[0]
    start = temp_path[0]
    for i in range(1,len(temp_path)):
        rst += get_distance(start,temp_path[i])
        start = temp_path[i]
    return rst

def get_distance(loc1, loc2):
    sx, sy = loc1
    gx, gy = loc2
    dx = gx-sx
    dy = gy-sy
    return round(math.sqrt(dx**2 + dy**2), res)

def compute_heuristics(roadmap, samples, goal):
    # Use Dikstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    goal_index = samples.index(goal)
    root = {'loc': goal, 'index': goal_index, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, goal_index, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, ind, curr) = heapq.heappop(open_list)
        connect_list = roadmap[ind]
        for connect_ind in connect_list:
            child_loc = samples[connect_ind]
            child_cost = round(cost + get_distance(loc, child_loc), res)
            child = {'loc': child_loc, 'index': connect_ind, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node[index], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, connect_ind, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, connect_ind, child))
    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

def build_constraint_table(constraints, agent):
    ##############################
    # Return a table that constains the list of constraints of
    # the given agent for each time step. The table can be used
    # for a more efficient constraint violation check in the
    # is_constrained function.
    #
    # Input : constraint(list of dict), agent id
    #     ex: {'agent': 2, 'loc': [(2,3)], 'timestep': 5}
    # Output: constraint table (dict)
    #     ex: {2: [[(2,3)], [(4,5)], [(2,3),(3,4)] ...]

    constraint_table = dict()
    if agent == 1:
        print(constraints)
    for constraint in constraints:
        if constraint['agent'] == agent:
            time_key = constraint_table.get(constraint['timestep'])
            if time_key is None:
                constraint_table[constraint['timestep']] = [constraint['loc']]
            else:
                time_key.append(constraint['loc'])
                constraint_table[constraint['timestep']] = time_key
    return constraint_table

def build_timeline(constraints):
    timeline = []
    new_const = deepcopy(constraints)
    for i, constraint in enumerate(constraints):
        for key, ele in constraint.items():
            timeline.append(key[0])
            timeline.append(key[1])
        new_const[i][(key[1], key[1])] = ele

    timeline = list(set(timeline))
    timeline.sort()
    return timeline, new_const

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]

def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        path.append(curr['timestep'])
        curr = curr['parent']
    path.reverse()
    return path

def is_constrained(curr_loc, next_loc, curr_time, child_cost, constraints):
    curr_lower = curr_time
    curr_upper = curr_time + child_cost
    if curr_loc == next_loc:
        curr_obs = Point(curr_loc).buffer(0.3)
    else:
        curr_obs = LineString([curr_loc, next_loc]).buffer(0.3)
    a = 2
    for const in constraints:
        for time, obs in const.items():
            lower_bound = time[0]
            upper_bound = time[1]

            if curr_upper < lower_bound or curr_lower > upper_bound:
                a = 0
            else:
                if curr_obs.intersects(obs):
                    return True
    return False

def push_node(open_list, node):
    heapq.heappush(open_list, (round(node['g_val'] + node['h_val'], res), round(node['h_val'], res), node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_node(n1, n2):
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def a_star(roadmap, samples, start_loc, goal_loc, h_values, agent, constraints, start_time, next_action):
    """
    roadamp, samples - map based probablistic roadmap
    start_loc - start position
    goal_loc - goal position
    agent - the agent that is being re-planned
    constraints - constraints defining where robot should or cannot go at each timestep
    """

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    time_line, new_const = build_timeline(constraints)
    last_time = None
    if time_line:
        last_time = time_line[-1]

    start_index = samples.index(start_loc)
    curr_time = start_time
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': round(curr_time, res), 'index': start_index, 'realtime': curr_time}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    maximum_node_bound = 200000
    node_counter = 0

    while len(open_list) > 0:

        curr = pop_node(open_list)
        ind = curr['index']
        connect_list = roadmap[ind]
        curr_loc = curr['loc']
        curr_time = curr['timestep']
        next_time = None
        curr_real = curr['realtime']
        for ele in time_line:
            if ele > curr_real:
                next_time = ele
                break
        ###########################################
        if node_counter > maximum_node_bound:
            print("Bound exceeds")
            return None
            raise BaseException('Bound exceeds')
        else:
            node_counter += 1
        # find goal
        if time_line:
            if curr['loc'] == goal_loc:
                if next_action == -1:	# the last goal -navi case
                    if curr_real >= last_time:
                        return get_path(curr)
                elif next_action < 0:  # the last goal - action case
                    if curr_real + abs(next_action) >= last_time:
                        if not is_constrained(curr['loc'], curr['loc'], curr_real, abs(next_action), new_const):
                            return get_path(curr)
                else:
                    if not is_constrained(curr['loc'], curr['loc'], curr_real, next_action, new_const):
                        return get_path(curr)

        else: # First agent case
            if curr['loc'] == goal_loc:
                return get_path(curr)
        ###########################################
        for connect_ind in connect_list:
            child_loc = samples[connect_ind]
            child_cost = get_distance(curr_loc, child_loc)
            child_cost_round = round(child_cost,res)
            if is_constrained(curr_loc, child_loc, curr_real, child_cost, new_const):
                continue

            child = {'loc': child_loc,
                     'g_val': round(curr['g_val'] + child_cost_round, res),
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': round(curr_time + child_cost_round, res),
                     'index': connect_ind,
                     'realtime': curr_real + child_cost}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_node(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)


        if last_time:
            if next_time:
                child_cost = next_time - curr_real
            else:
                child_cost = 1.0
            if is_constrained(curr_loc, curr_loc, curr_real, child_cost, new_const):
                pass
            else:
                child = {'loc': curr_loc,
                        'g_val': round(curr['g_val'] + child_cost, res),
                        'h_val': h_values[curr_loc],
                        'parent': curr,
                        'timestep': round(curr_time + child_cost, res),
                        'index': ind,
                        'realtime': curr_real + child_cost}
                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_node(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)


    return None           # Failed to find solutions
