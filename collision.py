import random
import copy
import time
from shapely.geometry import Point, LineString
from utils import *

def is_narrow_col_with_obstacles(col, prm):
    """
    Check Collision is Narrow col or not
    """
    ways = col[0]
    obstacles = prm.env.obstacles
    bounds = prm.bounds
    max_rr = prm.max_rr
    rr_margin = max_rr/3
    n_ways = len(ways)

    narrow_count  = 0
    for way in ways:
        count = 0
        waypoint = Point(way)
        for obstacle in obstacles:
            if obstacle.distance(waypoint) < max_rr*2 + rr_margin*1.5:
                count += 1
            if count >1:
                narrow_count += 1
                break
        if count < 2:
            if way[0] - bounds[0] < max_rr*2 + rr_margin or bounds[2] - way[0] < max_rr*2 + rr_margin:
                count += 1
            if way[1] - bounds[1] < max_rr*2 + rr_margin or bounds[3] - way[1] < max_rr*2 + rr_margin:
                count += 1
            if count > 1:
                narrow_count += 1

    if narrow_count > (n_ways/3):
        return True

    return False

def collision_check(robot_path, robots, prm, col_env):
    """
    Collision Check Function
    Robot path, robots: dictionary
    """
    collision = []
    key_list = list(robot_path)
    robot_num = len(key_list)
    collision_time = time.time()
    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            robot1_name = key_list[i]
            robot2_name = key_list[j]
            robot1_path = robot_path[robot1_name]
            robot2_path = robot_path[robot2_name]
            robot1_rr = robots[robot1_name].robot_radius
            robot2_rr = robots[robot2_name].robot_radius
            collision = collision + make_collision_line(prm, robot1_path, robot1_rr, robot2_path, robot2_rr)
    collision_elapsed_time = time.time() - collision_time

    # Late Token case
    if col_env == -1:
        return collision, -1
    narrow_col = []
    normal_col = []
    pick_collision = None
    for ele in collision:
        if is_narrow_col_with_obstacles(ele, prm):
            ele.append(0)
            narrow_col.append(ele)
        else:
            ele.append(1)
            normal_col.append(ele)

    # Randomly Choose one collision
    if narrow_col:
        n = len(narrow_col)
        pick_collision = narrow_col[random.randint(0,n-1)]
    elif normal_col:
        n = len(normal_col)
        pick_collision = normal_col[random.randint(0,n-1)]

    unity_col = copy.deepcopy(col_env)
    if pick_collision is not None:
        unity_col = handle_collision(prm, pick_collision, col_env)
    #print(unity_col)

    return collision, unity_col

def handle_collision(prm, collision, col_env):
    """
    collision = [[samples...], 0 or 1]
    col_env = [[two or greater endpoints], [samples], ], [...], [...]]
    """
    max_rr = prm.max_rr
    max_samples = prm.max_samples

    union_list = []
    copy_col = copy.deepcopy(collision)

    # overlap check
    # This time only check between new collision region and (prev collision entry and region)
    col_samples = copy_col[0]
    buffer_samples = []
    for i, prev in enumerate(col_env):
        prev_entry = prev[0]
        prev_samples = prev[1]
        prev_buffer = []
        for ele in prev_entry:
            if len(ele) == 1:
                prev_buffer.append(Point(ele[0]).buffer(max_rr))
            else:
                for j in range(1, len(ele)):
                    prev_buffer.append(LineString([ele[0], ele[j]]).buffer(max_rr))
        for col in col_samples:
            if i in union_list:
                break
            col_buf = Point(col).buffer(max_rr)
            for buf in prev_buffer:
                if buf.intersects(col_buf):
                    union_list.append(i)
                    for sam in max_samples:
                        samp_p = Point(sam)
                        if buf.contains(samp_p):
                            buffer_samples.append(sam)
                    break
            if col in prev_samples:
                union_list.append(i)
                break

    # If there is overlapped session, combine that, else only current collision will be handled
    temp_col_env = []
    if union_list:
        temp_sample = copy_col[0]
        if buffer_samples:
            # print("wuwang: ", buffer_samples)
            temp_sample += buffer_samples
        # additional_sample = copy.deepcopy(collision[0])
        temp_ind = [copy_col[1]]
        new_col = []
        # ex_end = []
        for i in range(len(col_env)):
            if not i in union_list:
                temp_col_env.append(col_env[i])
            else:
                temp_sample += col_env[i][1]
                temp_ind.append(col_env[i][2])

        col_ind = 0
        if len(temp_ind) == sum(temp_ind):
            col_ind = 1
        new_col = [list(set(temp_sample)), col_ind]
        # new_col = handle_endpoint(prm, new_col, list(set(ex_end)))
        new_col = handle_endpoint(prm, new_col)
        temp_col_env.append(new_col)
    else:
        new_col = handle_endpoint(prm, copy_col)
        temp_col_env = copy.deepcopy(col_env)
        temp_col_env.append(new_col)

    # Overlap check for generated entry points
    # Iterate unitl there is no overlap.
    while True:
        union_list = []
        entry_buffer = []
        check_entry = temp_col_env[-1][0]
        check_samples = temp_col_env[-1][1]
        check_ind = temp_col_env[-1][2]
        buffer_samples = []
        for ele in check_entry:
            if len(ele) == 1:
                entry_buffer.append(Point(ele[0]).buffer(max_rr))
            else:
                for i in range(1, len(ele)):
                    entry_buffer.append(LineString([ele[0], ele[i]]).buffer(max_rr))

        # Checking process
        for i in range(len(temp_col_env)-1):
            current_buffer = []
            current_samples = temp_col_env[i][1]
            current_entry = temp_col_env[i][0]
            for ele in current_entry:
                if len(ele) == 1:
                    current_buffer.append(Point(ele[0]).buffer(max_rr))
                else:
                    for j in range(1, len(ele)):
                        current_buffer.append(LineString([ele[0], ele[j]]).buffer(max_rr))
            for entry_b in entry_buffer:
                if i in union_list:
                    break
                for buf in current_buffer:
                    if entry_b.intersects(buf):
                        union_list.append(i)
                        for sam in max_samples:
                            samp_p = Point(sam)
                            if buf.contains(samp_p) or entry_b.contains(samp_p):
                                buffer_samples.append(sam)
                        break
                for col in current_samples:
                    col_buf = Point(col).buffer(max_rr)
                    if entry_b.intersects(col_buf):
                        union_list.append(i)
                        for sam in max_samples:
                            samp_p = Point(sam)
                            if col_buf.contains(samp_p) or entry_b.contains(samp_p):
                                buffer_samples.append(sam)
                        break
        # Integrate process
        if union_list:
            temp_temp_col_env = []
            temp_sample = copy.deepcopy(check_samples)
            if buffer_samples:
                # print("wuwang2: ", buffer_samples)
                temp_sample += buffer_samples
            temp_ind = [check_ind]
            new_col = []
            # ex_end = []
            for i in range(len(temp_col_env)-1):
                if not i in union_list:
                    temp_temp_col_env.append(temp_col_env[i])
                else:
                    temp_sample += temp_col_env[i][1]
                    temp_ind.append(temp_col_env[i][2])

            col_ind = 0
            if len(temp_ind) == sum(temp_ind):
                col_ind = 1
            new_col = [list(set(temp_sample)), col_ind]
            new_col = handle_endpoint(prm, new_col)
            temp_temp_col_env.append(new_col)
            temp_col_env = temp_temp_col_env
        else:
            break

    return temp_col_env

def handle_endpoint(prm, new_col):
    new_entry = None
    max_rr = prm.max_rr
    max_samples = prm.max_samples

    # Make Roadmap without collision samples
    c_roadmap_samples = []
    c_roadmap = []
    for sample in max_samples:
        if not sample in new_col[0]:
            c_roadmap_samples.append(sample)

    # Separate internal waypoints and external waypoints
    important = []
    delete_vor_way = []
    new_vor_way = []
    external_imp = []
    internal_imp = []

    for robot, ele in prm.robots_map.items():
        important.append(ele.pos)   # Robot pos
    for ele in prm.goal_samples:
        important += ele            # Goal pos
    for ele in prm.vor_ways:
        if ele in new_col[0]:
            delete_vor_way.append(ele)
        else:
            external_imp.append(ele)    # Voronoi pos

    for ele in important:
        if ele in new_col[0]:
            internal_imp.append(ele)
        else:
            external_imp.append(ele)

    # 1. Make End Points

    endpoint_time = time.time()
    col_samples = new_col[0]
    sample_num = len(col_samples)
    temp_end = []

    buffered_end = []
    delete_samples = []
    start_samples = []

    for ele in internal_imp:
        buffered_end.append(Point(ele).buffer(max_rr*3.0)) #VRP CHANGE 3.0 -> 4.0
        temp_end.append(ele)

    for sample in col_samples:
        p_sample = Point(sample)
        for ele in buffered_end:
            if ele.contains(p_sample):
                delete_samples.append(sample)
                break
    for sample in col_samples:
        if not sample in delete_samples:
            start_samples.append(sample)

    while True:
        if not start_samples:
            break
        sample_num = len(start_samples)
        if sample_num == 1:
            temp_end.append(start_samples[0])
            break
        else:
            a = None
            b = None
            max_dist = 0
            temp_start = []
            for i in range(sample_num-1):
                temp_a = start_samples[i]
                for j in range(i+1, sample_num):
                    temp_b = start_samples[j]
                    if get_distance(temp_a, temp_b) > max_dist:
                        max_dist = get_distance(temp_a, temp_b)
                        a = temp_a
                        b = temp_b
            if max_dist > max_rr * 4.0: # VRP 4.0 -> 6.0
                temp_end.append(a)
                temp_end.append(b)
                buffered_a = Point(a).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                buffered_b = Point(b).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                for sample in start_samples:
                    p_sample = Point(sample)
                    if not buffered_a.contains(p_sample) and not buffered_b.contains(p_sample):
                        temp_start.append(sample)
                start_samples = temp_start
            else:
                min_dist = 100000
                max_num_sam = -1
                end_candidate = None
                for ele1 in start_samples:
                    temp_num_sam = 0
                    buf_cand = Point(ele1).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                    for ele2 in c_roadmap_samples:
                        p_sam = Point(ele2)
                        if buf_cand.contains(p_sam):
                            temp_num_sam += 1
                    if max_num_sam < temp_num_sam:
                        max_num_sam = temp_num_sam
                        end_candidate = ele1

                if end_candidate:
                    temp_end.append(end_candidate)
                    buffered_candidate = Point(end_candidate).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                    for sample in start_samples:
                        p_sample = Point(sample)
                        if not buffered_candidate.contains(p_sample):
                            temp_start.append(sample)
                    start_samples = temp_start
                else:
                    break

    for ele in temp_end:
        for ele2 in prm.vor_ways:
            if get_distance(ele, ele2) < max_rr*2.5:
                delete_vor_way.append(ele2)



    # 2. Make entry points

    if new_col[1] == 1:
        col_end = []
        for ele in temp_end:
            col_end.append([ele])
        new_entry = [col_end, col_samples, new_col[1]]
    else:
        #c_roadmap_samples, temp_end
        temp_nears = []
        near_samples = {}
        for sample in c_roadmap_samples:
            p_sample = Point(sample)
            near_flag = False
            for ele in temp_end:
                end_buf = Point(ele).buffer(max_rr*4.5)
                if end_buf.contains(p_sample):
                    near_flag = True
                    break
            if near_flag:
                temp_nears.append(sample)

        for sample in temp_nears:
            min_dist = 100000
            min_end = None
            for ele in temp_end:
                if get_distance(sample, ele) < min_dist:
                    min_dist = get_distance(sample,ele)
                    min_end = ele
            if min_dist > max_rr *2 or (sample in external_imp and not sample in prm.vor_ways):
                near_samples[min_end] = near_samples.get(min_end, []) + [sample]

        # print(near_samples, len(near_samples))

        temp_entry = []
        entry_dict = {}

        for ele in temp_end:
            entry_candidates = copy.deepcopy(near_samples.get(ele, []))
            temp_external = []
            entry_dict[ele] = []
            # For important way as a entry
            for ele2 in entry_candidates:
                if ele2 in external_imp and not ele2 in prm.vor_ways:
                    temp_entry.append(ele2)
                    temp_external.append(ele2)
                    entry_dict[ele] = entry_dict.get(ele, []) + [ele2]
            temp_entry_candidates = []
            for ele2 in entry_candidates:
                p = Point(ele2)
                imp_flag = False
                for ele3 in temp_external:
                    if Point(ele3).buffer(max_rr*3.0).contains(p): # scene1 3.0 - > 2.5
                        imp_flag = True
                        break
                if not imp_flag:
                    temp_entry_candidates.append(ele2)
            entry_candidates = temp_entry_candidates

            while True:
                candi_len = len(entry_candidates)
                if candi_len == 0:
                    break
                elif candi_len == 1:
                    temp_entry.append(entry_candidates[0])
                    entry_dict[ele] = entry_dict.get(ele, []) + [entry_candidates[0]]
                    break
                else:
                    a = None
                    b = None
                    max_dist = 0
                    temp_start = []
                    for i in range(candi_len-1):
                        temp_a = entry_candidates[i]
                        for j in range(i+1, candi_len):
                            temp_b = entry_candidates[j]
                            if get_distance(temp_a, temp_b) > max_dist:
                                max_dist = get_distance(temp_a, temp_b)
                                a = temp_a
                                b = temp_b
                    if max_dist > max_rr * 2.5:
                        temp_entry.append(a)
                        temp_entry.append(b)
                        entry_dict[ele] = entry_dict.get(ele, []) + [a]
                        entry_dict[ele] = entry_dict.get(ele, []) + [b]
                        buffered_a = Point(a).buffer(max_rr*3.0) # scene1 3.0 - > 2.5
                        buffered_b = Point(b).buffer(max_rr*3.0) # scene1 3.0 - > 2.5
                        for sample in entry_candidates:
                            p_sample = Point(sample)
                            if not buffered_a.contains(p_sample) and not buffered_b.contains(p_sample):
                                temp_start.append(sample)
                        entry_candidates = temp_start
                    else:
                        min_dist = 100000
                        one_entry = None
                        for ele2 in entry_candidates:
                            if get_distance(ele, ele2) < min_dist:
                                min_dist = get_distance(ele, ele2)
                                one_entry = ele2
                        if one_entry:
                            temp_entry.append(one_entry)
                            entry_dict[ele] = entry_dict.get(ele, []) + [one_entry]
                            buffered_candidate = Point(one_entry).buffer(max_rr*3.0) # scene1 3.0 - > 2.5
                            for sample in entry_candidates:
                                p_sample = Point(sample)
                                if not buffered_candidate.contains(p_sample):
                                    temp_start.append(sample)
                            entry_candidates = temp_start
                        else:
                            break

        # Vornoi delete
        for ele in temp_entry:
            for ele2 in prm.vor_ways:
                if get_distance(ele, ele2) < max_rr*2.5:
                    delete_vor_way.append(ele2)

        find_entry = []
        for name, description in entry_dict.items():
            temp_find = [name]
            temp_find += description
            find_entry.append(temp_find)

        new_entry = [find_entry, new_col[0], new_col[1]]

    # Setting The Voronoi Sample for the save
    for ele in prm.vor_ways:
        if not ele in delete_vor_way:
            new_vor_way.append(ele)

    prm.vor_ways = new_vor_way

    return new_entry

def make_collision_line(prm, path1, rr1, path2, rr2):
    """
    Waiting Timing must be added later.
    """
    path1_cost = path1['path_cost']
    path2_cost = path2['path_cost']
    max_time = int(max(path1['path_time'], path2['path_time']) * 10)

    # agent 1
    path_with_time1 = []
    temp_time = 0.0
    path_len = len(path1_cost)
    starts = None
    if path_len > 1:
        for i in range(1, path_len):
            if i == 1:
                if type(path1_cost[i-1]) == list:
                    for partial in path1_cost[i-1][2]:
                        if partial[1] == 2:
                            path_with_time1.append((path1_cost[i-1][0], temp_time))
                            temp_time += round(partial[0], 2)
                        else:
                            path_with_time1.append((path1_cost[i-1][0], temp_time, partial[1], partial[2]))
                            temp_time += round(partial[0], 2)
                    path_with_time1.append((path1_cost[i-1][0], temp_time))
                    starts = path1_cost[i-1][0]
                else:
                    path_with_time1.append((path1_cost[i-1], temp_time))
                    starts = path1_cost[i-1]
            if type(path1_cost[i]) == list:
                temp_time += round(get_distance(starts, path1_cost[i][0]), 2)
                path_with_time1.append((path1_cost[i][0], temp_time))
                for partial in path1_cost[i][2]:
                    if partial[1] == 2:
                        temp_time += round(partial[0], 2)
                        path_with_time1.append((path1_cost[i][0], temp_time))
                    else:
                        temp_time += round(partial[0], 2)
                        path_with_time1.append((path1_cost[i][0], temp_time, partial[1], partial[2]))
                starts = path1_cost[i][0]
            else:
                temp_time += round(get_distance(starts, path1_cost[i]), 2)
                path_with_time1.append((path1_cost[i], temp_time))
                starts = path1_cost[i]
    else:
        path_with_time1 = [(path1_cost[0][0], temp_time)]

    # agent 2
    path_with_time2 = []
    temp_time = 0.0
    path_len = len(path2_cost)
    starts = None
    if path_len > 1:
        for i in range(1, path_len):
            if i == 1:
                if type(path2_cost[i-1]) == list:
                    for partial in path2_cost[i-1][2]:
                        if partial[1] == 2:
                            path_with_time2.append((path2_cost[i-1][0], temp_time))
                            temp_time += round(partial[0], 2)
                        else:
                            path_with_time2.append((path2_cost[i-1][0], temp_time, partial[1], partial[2]))
                            temp_time += round(partial[0], 2)
                    path_with_time2.append((path2_cost[i-1][0], temp_time))
                    starts = path2_cost[i-1][0]
                else:
                    path_with_time2.append((path2_cost[i-1], temp_time))
                    starts = path2_cost[i-1]
            if type(path2_cost[i]) == list:
                temp_time += round(get_distance(starts, path2_cost[i][0]), 2)
                path_with_time2.append((path2_cost[i][0], temp_time))
                for partial in path2_cost[i][2]:
                    if partial[1] == 2:
                        temp_time += round(partial[0], 2)
                        path_with_time2.append((path2_cost[i][0], temp_time))
                    else:
                        temp_time += round(partial[0], 2)
                        path_with_time2.append((path2_cost[i][0], temp_time, partial[1], partial[2]))
                starts = path2_cost[i][0]
            else:
                temp_time += round(get_distance(starts, path2_cost[i]), 2)
                path_with_time2.append((path2_cost[i], temp_time))
                starts = path2_cost[i]
    else:
        path_with_time2 = [(path2_cost[0][0], temp_time)]

    collision = []

    temp_col = []
    prev_col = []

    timestep = 0
    for timestep in range(0, max_time):
        pos1 = get_state_time(timestep / 10, path_with_time1)
        pos2 = get_state_time(timestep / 10, path_with_time2)
        if get_distance(pos1, pos2) < rr1 + rr2:
            prev_col.append(pos1)
            prev_col.append(pos2)
        else:
            if prev_col:
                temp_col.append(prev_col)
                prev_col = []

    samples = prm.max_samples
    max_rr = prm.max_rr

    for ele in temp_col:
        collision_sample = []
        buffered_list = []

        for pos in ele:
            buffered_pos = Point(pos).buffer(max_rr*2.0)
            buffered_list.append(buffered_pos)

        for sample in samples:
            sp = Point(sample)
            for buffered in buffered_list:
                if buffered.contains(sp):
                    collision_sample.append(sample)
                    break
        collision.append([collision_sample])

    return collision

def get_state_time(t, path):
    if t <= path[0][1]:
        return path[0][0]
    elif t >= path[-1][1]:
        return path[-1][0]
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

        if dx ==0 and dy == 0:
            return (x2, y2)

        pos_x = x1 + (t - path[i][1]) * 1 * math.cos(yaw)
        pos_y = y1 + (t - path[i][1]) * 1 * math.sin(yaw)
        pos = (pos_x, pos_y)

        return pos
