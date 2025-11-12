import utils
import random
import sys
from collections import deque, defaultdict  
import heapq

# 设置递归深度以应对大型地图中的可达性搜索
sys.setrecursionlimit(2000) 

# 方向常量，便于计算
DIRECTIONS = {
    '^': (0, -1),   # 上 (Y轴负方向)
    '>': (1, 0),    # 右 (X轴正方向)
    'v': (0, 1),    # 下 (Y轴正方向)
    '<': (-1, 0)    # 左 (X轴负方向)
}

TURN_LEFT = {
    '^': '<',
    '<': 'v',
    'v': '>',
    '>': '^'
}

TURN_RIGHT = {
    '^': '>',
    '>': 'v',
    'v': '<',
    '<': '^'
}

def move_with_wrap(pos, direction, map_grid):
    """
    根据当前方向和地图，计算移动一步后的新位置（处理环绕）。
    
    Args:
        pos (tuple): 当前位置 (x, y)。
        direction (str): 当前方向。
        map_grid (list[list[str]]): 地图的二维列表。

    Returns:
        tuple: 新位置 (x, y)。
    """
    dx, dy = DIRECTIONS[direction]
    x, y = pos
    new_x, new_y = x + dx, y + dy

    # 处理环绕 (No Boundary)
    height = len(map_grid)
    width = len(map_grid[0]) if height > 0 else 0
    new_x = new_x % width
    new_y = new_y % height

    return (new_x, new_y)

def simulate_robot(start_pos, start_dir, plan, map_grid):
    """
    模拟机器人在给定初始状态下执行计划。
    
    Args:
        start_pos (tuple): 起始位置 (x, y)。
        start_dir (str): 起始方向 ('^', '>', 'v', '<')。
        plan (str): 计划指令字符串 (e.g., 'MLMRM')。
        map_grid (list[list[str]]): 地图的二维列表。

    Returns:
        set: 机器人在执行计划过程中访问过的所有格子坐标的集合。
    """
    if not plan:
        return {start_pos}

    current_pos = start_pos
    current_dir = start_dir
    # 集合存储访问过的位置 (x, y)
    visited = {current_pos} 

    for instruction in plan:
        if instruction == 'L':
            current_dir = TURN_LEFT[current_dir]
        elif instruction == 'R':
            current_dir = TURN_RIGHT[current_dir]
        elif instruction == 'M':
            new_pos = move_with_wrap(current_pos, current_dir, map_grid)
            
            # 检查新位置是否是墙。如果是墙，机器人停在原地。
            if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                current_pos = new_pos
                visited.add(current_pos)
        # 忽略其他无效指令

    return visited

def get_reachable_cleanable_spaces(map_grid, start_pos):
    """
    使用 BFS 或 DFS 找到所有可达的、需要清理的空地 (' ') 坐标。
    """
    height = len(map_grid)
    width = len(map_grid[0])
    queue = [start_pos]
    reachable = {start_pos}
    
    # 确保起点本身不是墙
    if utils.is_wall(map_grid, start_pos[0], start_pos[1]):
        return set()

    # 简化的 BFS，只考虑四个基本方向的移动
    # 注意：这里的移动是逻辑上的"可达"，需要考虑环绕，但不能穿墙。
    while queue:
        x, y = queue.pop(0)
        
        # 遍历四个方向的相邻格子
        for dx, dy in DIRECTIONS.values():
            # 计算环绕后的相邻坐标
            next_x = (x + dx) % width
            next_y = (y + dy) % height
            next_pos = (next_x, next_y)
            
            if next_pos not in reachable:
                # 只有非墙体才能移动到
                if not utils.is_wall(map_grid, next_x, next_y):
                    reachable.add(next_pos)
                    queue.append(next_pos)
    
    # 筛选出其中需要清理的空地 (' ')
    cleanable_and_reachable = set()
    for x, y in reachable:
        if map_grid[y][x] == ' ':
            cleanable_and_reachable.add((x, y))
            
    return cleanable_and_reachable


def check_plan(problem_data):
    """
    检查给定的计划是否能清理所有可达的空格子。
    核心：处理"未知朝向"时，采用访问格子的交集 (Intersection)。
    
    Args:
        problem_data (dict): 从 utils.load_problem_file 加载的数据。

    Returns:
        str: 解决方案文本，格式为 "GOOD PLAN" 或 "BAD PLAN\nx1, y1\nx2, y2\n..."
    """
    if problem_data['type'] != 'CHECK_PLAN':
        raise ValueError("check_plan function called on a non-CHECK_PLAN problem.")

    map_grid = problem_data['map']
    plan = problem_data['plan']
    start_pos = problem_data['start_pos']
    start_dirs = problem_data['start_dirs']

    # 1. 找到所有可达的、需要清理的空地 (' ')
    # 这一步非常重要，因为只有可达的空格才需要被清理
    all_target_spaces = get_reachable_cleanable_spaces(map_grid, start_pos)
    
    if not all_target_spaces:
        # 如果没有需要清理的空地，则计划总是好的
        return "GOOD PLAN"

    # 2. 模拟执行计划并计算"确保已清理"的格子集合
    
    # 初始化确保已清理集合为第一次模拟的结果
    # 这样可以方便后续使用集合的交集操作
    sure_cleaned_spaces = simulate_robot(start_pos, start_dirs[0], plan, map_grid)
    
    # 对剩余的起始方向进行模拟，并计算交集
    # 如果只有一个起始方向（问题A, B），循环将不执行，结果就是第一次模拟
    # 如果有多个起始方向（问题C），结果是所有模拟路径的交集
    for direction in start_dirs[1:]:
        visited = simulate_robot(start_pos, direction, plan, map_grid)
        sure_cleaned_spaces = sure_cleaned_spaces.intersection(visited) 
        # ⚠️ 修正：确保已清理的集合是所有模拟路径的交集！

    # 3. 找出未被确保清理的空格子
    # 仅考虑目标格子集合 (all_target_spaces) 中那些不在 sure_cleaned_spaces 中的格子
    missed_spaces = all_target_spaces - sure_cleaned_spaces

    # 4. 格式化输出
    if not missed_spaces:
        return "GOOD PLAN"
    else:
        # 格式：第一行 "BAD PLAN"，后续每行 "x, y"
        lines = ["BAD PLAN"]
        # 结果必须按坐标排序以保证输出一致性
        # 排序键：先按 y 坐标，再按 x 坐标
        for x, y in sorted(missed_spaces, key=lambda p: (p[1], p[0])):
            lines.append(f"{x}, {y}")
        return "\n".join(lines)


# ==================== FIND PLAN 相关函数 (重新设计) ====================

def find_shortest_path_between(start, goal, map_grid):
    """
    使用BFS找到从start到goal的最短路径（仅位置，不考虑方向）。
    返回移动序列和到达goal时可能的方向。
    """
    height = len(map_grid)
    width = len(map_grid[0])
    
    queue = deque([(start, [])])
    visited = {start}
    
    while queue:
        pos, path = queue.popleft()
        
        if pos == goal:
            return path
        
        for direction, (dx, dy) in DIRECTIONS.items():
            next_x = (pos[0] + dx) % width
            next_y = (pos[1] + dy) % height
            next_pos = (next_x, next_y)
            
            if next_pos not in visited and not utils.is_wall(map_grid, next_x, next_y):
                visited.add(next_pos)
                queue.append((next_pos, path + [direction]))
    
    return None


def turn_to_direction(current_dir, target_dir):
    """
    返回从current_dir转向target_dir所需的指令。
    """
    if current_dir == target_dir:
        return ""
    
    # 计算最短旋转路径
    dirs = ['^', '>', 'v', '<']
    current_idx = dirs.index(current_dir)
    target_idx = dirs.index(target_dir)
    
    diff = (target_idx - current_idx) % 4
    
    if diff == 1:
        return "R"
    elif diff == 2:
        return "RR"
    elif diff == 3:
        return "L"
    
    return ""


def find_plan_greedy_nearest(problem_data):
    """
    贪心算法：每次前往最近的未清理格子。
    使用BFS路径规划确保能到达。
    """
    map_grid = problem_data['map']
    start_pos = problem_data['start_pos']
    start_dir = problem_data['start_dirs'][0]
    
    target_spaces = get_reachable_cleanable_spaces(map_grid, start_pos)
    
    if not target_spaces:
        return ""
    
    current_pos = start_pos
    current_dir = start_dir
    plan = ""
    cleaned = set()
    
    # 如果起始位置是空格，加入已清理
    if map_grid[start_pos[1]][start_pos[0]] == ' ':
        cleaned.add(start_pos)
    
    remaining = target_spaces - cleaned
    
    # 防止无限循环
    max_iterations = len(target_spaces) * 100
    iterations = 0
    
    while remaining and iterations < max_iterations:
        iterations += 1
        
        # 找到最近的未清理格子
        nearest = None
        min_dist = float('inf')
        
        for target in remaining:
            dist = abs(target[0] - current_pos[0]) + abs(target[1] - current_pos[1])
            if dist < min_dist:
                min_dist = dist
                nearest = target
        
        if nearest is None:
            break
        
        # 找到前往nearest的路径
        path_directions = find_shortest_path_between(current_pos, nearest, map_grid)
        
        if path_directions is None:
            # 无法到达，移除这个目标
            remaining.remove(nearest)
            continue
        
        # 执行路径
        for target_dir in path_directions:
            # 转向
            turn_plan = turn_to_direction(current_dir, target_dir)
            plan += turn_plan
            for t in turn_plan:
                if t == 'L':
                    current_dir = TURN_LEFT[current_dir]
                elif t == 'R':
                    current_dir = TURN_RIGHT[current_dir]
            
            # 移动
            plan += "M"
            new_pos = move_with_wrap(current_pos, current_dir, map_grid)
            if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                current_pos = new_pos
                if current_pos in target_spaces:
                    cleaned.add(current_pos)
        
        remaining = target_spaces - cleaned
    
    return plan


def find_plan_bfs_limited(problem_data, max_states=50000):
    """
    限制状态数的BFS搜索。
    """
    map_grid = problem_data['map']
    start_pos = problem_data['start_pos']
    start_dir = problem_data['start_dirs'][0]
    
    target_spaces = get_reachable_cleanable_spaces(map_grid, start_pos)
    
    if not target_spaces:
        return ""
    
    # 检查起始位置
    start_x, start_y = start_pos
    initial_cleaned_set = set()
    if map_grid[start_y][start_x] == ' ':
        initial_cleaned_set.add(start_pos)
    initial_cleaned = frozenset(initial_cleaned_set)
    
    queue = deque([(start_pos, start_dir, initial_cleaned, "")])
    visited = {(start_pos, start_dir, initial_cleaned)}
    
    states_explored = 0
    
    while queue and states_explored < max_states:
        pos, direction, cleaned, plan = queue.popleft()
        states_explored += 1
        
        if target_spaces.issubset(cleaned):
            return plan
        
        # 优先尝试移动到未清理的格子
        for action in ['M', 'L', 'R']:
            new_pos = pos
            new_dir = direction
            new_cleaned = set(cleaned)
            
            if action == 'L':
                new_dir = TURN_LEFT[direction]
            elif action == 'R':
                new_dir = TURN_RIGHT[direction]
            elif action == 'M':
                new_pos = move_with_wrap(pos, direction, map_grid)
                
                if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                    if new_pos in target_spaces:
                        new_cleaned.add(new_pos)
                else:
                    new_pos = pos
            
            new_cleaned_frozen = frozenset(new_cleaned)
            state = (new_pos, new_dir, new_cleaned_frozen)
            
            if state not in visited:
                visited.add(state)
                queue.append((new_pos, new_dir, new_cleaned_frozen, plan + action))
    
    return None

def find_shortest_path_with_direction(start_pos, start_dir, goal_pos, map_grid):
    """
    使用BFS找到从start_pos+start_dir到goal_pos的最短路径
    返回移动序列，确保能够实际到达目标位置
    """
    height = len(map_grid)
    width = len(map_grid[0])
    
    # 状态：(位置, 方向)
    queue = deque([(start_pos, start_dir, "")])
    visited = {(start_pos, start_dir)}
    
    while queue:
        pos, direction, path = queue.popleft()
        
        if pos == goal_pos:
            return path
        
        # 尝试三种动作：左转、右转、前进
        for action in ['L', 'R', 'M']:
            new_pos = pos
            new_dir = direction
            new_path = path + action
            
            if action == 'L':
                new_dir = TURN_LEFT[direction]
            elif action == 'R':
                new_dir = TURN_RIGHT[direction]
            elif action == 'M':
                temp_pos = move_with_wrap(pos, direction, map_grid)
                if not utils.is_wall(map_grid, temp_pos[0], temp_pos[1]):
                    new_pos = temp_pos
                else:
                    # 遇到墙，不能移动
                    continue
            
            state = (new_pos, new_dir)
            if state not in visited:
                visited.add(state)
                queue.append((new_pos, new_dir, new_path))
    
    return None

def find_plan(problem_data):
    """
    主函数: 为FIND_PLAN问题找到解决方案，处理方向不确定性
    """
    if problem_data['type'] != 'FIND_PLAN':
        raise ValueError("find_plan function called on a non-FIND_PLAN problem.")
    
    target_spaces = get_reachable_cleanable_spaces(problem_data['map'], 
                                                    problem_data['start_pos'])
    
    if not target_spaces:
        return ""
    
    # 如果只有一个起始方向，使用原有算法
    if len(problem_data['start_dirs']) == 1:
        # 选择适合单方向的算法
        num_targets = len(target_spaces)
        if num_targets <= 20:
            plan = find_plan_bfs_limited(problem_data, max_states=100000)
            if plan:
                return plan
        return find_plan_greedy_nearest(problem_data)
    
    # 多个起始方向：需要鲁棒性更强的策略
    return find_plan_robust_for_multiple_directions(problem_data)




def find_plan_robust_for_multiple_directions(problem_data):
    """
    为Problem F设计：使用蚁群算法 (ACO) 生成一个对所有起始方向都鲁棒的计划。
    """
    