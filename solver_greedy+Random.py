def find_plan_robust_for_multiple_directions(problem_data):
    """
    主函数：结合贪心、随机游走、迭代扩展，解决多起始方向问题。
    """
    map_grid = problem_data['map']
    start_pos = problem_data['start_pos']
    all_start_dirs = problem_data['start_dirs']
    target_spaces = get_reachable_cleanable_spaces(map_grid, start_pos)

    if not target_spaces:
        return ""

    # 1. 为 NORTH 方向生成基础 plan (使用贪心+随机游走策略)
    base_plan = generate_adaptive_plan_for_single_direction(problem_data, start_dir='^')
    current_plan = base_plan

    max_iterations = 200
    iteration = 0

    while iteration < max_iterations:
        iteration += 1
        all_good = True
        missed_spaces_for_any_dir = set()

        # 2. 模拟当前 plan 对所有起始方向的效果
        for dir in all_start_dirs:
            visited_by_this_dir = simulate_robot(start_pos, dir, current_plan, map_grid)
            missed_by_dir = target_spaces - visited_by_this_dir
            missed_spaces_for_any_dir.update(missed_by_dir)
            if missed_by_dir:
                all_good = False
                print(f"Iteration {iteration}: Plan fails for {dir}, missed {len(missed_by_dir)} spaces.")

        if all_good:
            print(f"Plan finalized after {iteration} iterations. Total length: {len(current_plan)}")
            return current_plan

        # 3. 扩展 plan：从某个方向的最终状态出发，生成补丁
        # 选择 NORTH 方向的最终状态作为起点
        final_pos_north, final_dir_north = get_final_state_after_plan(start_pos, '^', current_plan, map_grid)
        
        # 生成一个能访问所有遗漏格子的补丁 plan
        # 这里可以复用你之前设计的自适应策略（贪心+随机游走）
        patch_plan = generate_adaptive_plan_to_visit_targets(
            final_pos_north, final_dir_north, missed_spaces_for_any_dir, map_grid
        )

        if not patch_plan:
            # 如果无法生成补丁（理论上不应发生），使用随机游走作为备选
            patch_plan = generate_patch_plan_random_walk(final_pos_north, final_dir_north, map_grid, steps=20)

        # 4. 追加补丁到当前 plan
        current_plan += patch_plan
        print(f"Extended plan by {len(patch_plan)} steps in iteration {iteration}.")

    print(f"Warning: Plan not fully optimized after {max_iterations} iterations. Length: {len(current_plan)}")
    return current_plan


def generate_adaptive_plan_for_single_direction(problem_data, start_dir):
    """
    为单个起始方向生成 plan，使用“贪心+随机游走”策略。
    这是你的“局部策略”实现。
    """
    map_grid = problem_data['map']
    start_pos = problem_data['start_pos']
    target_spaces = get_reachable_cleanable_spaces(map_grid, start_pos)
    
    current_pos = start_pos
    current_dir = start_dir
    plan = ""
    cleaned = set()
    
    if map_grid[start_pos[1]][start_pos[0]] == ' ':
        cleaned.add(start_pos)

    remaining_targets = set(target_spaces)
    # 防止无限循环
    max_steps = len(target_spaces) * 100
    steps_taken = 0

    while remaining_targets and steps_taken < max_steps:
        steps_taken += 1

        # --- 核心逻辑：贪心 or 随机游走 ---
        # 检查周围4个方向是否还有未清理的格子
        neighbors_cleaned = True
        for dx, dy in DIRECTIONS.values():
            nx = (current_pos[0] + dx) % len(map_grid[0])
            ny = (current_pos[1] + dy) % len(map_grid)
            if map_grid[ny][nx] == ' ' and (nx, ny) not in cleaned:
                neighbors_cleaned = False
                break

        if neighbors_cleaned:
            # 僵局：周围都清理了，切换为随机游走
            print(f"Stuck at {current_pos}, switching to random walk.")
            # 随机走几步，直到找到新目标或达到步数上限
            random_steps = 0
            max_random_steps = 10
            while random_steps < max_random_steps and remaining_targets:
                action = random.choice(['L', 'R', 'M'])
                plan += action
                random_steps += 1
                steps_taken += 1

                if action == 'L':
                    current_dir = TURN_LEFT[current_dir]
                elif action == 'R':
                    current_dir = TURN_RIGHT[current_dir]
                elif action == 'M':
                    new_pos = move_with_wrap(current_pos, current_dir, map_grid)
                    if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                        current_pos = new_pos
                        if map_grid[current_pos[1]][current_pos[0]] == ' ':
                            cleaned.add(current_pos)
                            remaining_targets.discard(current_pos)
                            # 一旦找到新目标，立即切换回贪心
                            print(f"Found new target at {current_pos}, switching back to greedy.")
                            break
        else:
            # 未陷入僵局，使用贪心算法找最近目标
            nearest_target = None
            min_path = None
            for target in remaining_targets:
                path = find_shortest_path_with_direction(current_pos, current_dir, target, map_grid)
                if path is not None and (min_path is None or len(path) < len(min_path)):
                    min_path = path
                    nearest_target = target

            if not min_path: # 理论上不应发生
                break

            # 执行到最近目标的路径
            plan += min_path
            # 更新状态
            for instr in min_path:
                if instr == 'L': current_dir = TURN_LEFT[current_dir]
                elif instr == 'R': current_dir = TURN_RIGHT[current_dir]
                elif instr == 'M':
                    new_pos = move_with_wrap(current_pos, current_dir, map_grid)
                    if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                        current_pos = new_pos
                        if map_grid[current_pos[1]][current_pos[0]] == ' ':
                            cleaned.add(current_pos)
                            remaining_targets.discard(current_pos)
    return plan


def generate_adaptive_plan_to_visit_targets(from_pos, from_dir, targets_to_visit, map_grid):
    """
    从指定状态出发，生成一个 plan 访问所有目标点。
    同样使用“贪心+随机游走”策略。
    """
    if not targets_to_visit:
        return ""
    # 复用上面的自适应逻辑，但目标是特定的 targets_to_visit
    current_pos = from_pos
    current_dir = from_dir
    plan = ""
    cleaned = set() # 在这个子计划中访问过的点
    remaining_targets = set(targets_to_visit)
    max_steps = len(targets_to_visit) * 50
    steps_taken = 0

    while remaining_targets and steps_taken < max_steps:
        steps_taken += 1
        neighbors_cleaned = True
        for dx, dy in DIRECTIONS.values():
            nx = (current_pos[0] + dx) % len(map_grid[0])
            ny = (current_pos[1] + dy) % len(map_grid)
            if (nx, ny) in remaining_targets:
                neighbors_cleaned = False
                break

        if neighbors_cleaned:
            # 僵局：周围都清理了，切换为随机游走
            # 增加随机游走步数，从15步改为30步，以增强探索能力
            print(f"Stuck at {current_pos} while generating patch, switching to random walk.")
            # 随机走几步，直到找到新目标或达到步数上限
            random_steps = 0
            max_random_steps = 15  # 15步
            while random_steps < max_random_steps and remaining_targets:
                action = random.choice(['L', 'R', 'M'])
                plan += action
                random_steps += 1
                steps_taken += 1

                if action == 'L':
                    current_dir = TURN_LEFT[current_dir]
                elif action == 'R':
                    current_dir = TURN_RIGHT[current_dir]
                elif action == 'M':
                    new_pos = move_with_wrap(current_pos, current_dir, map_grid)
                    if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                        current_pos = new_pos
                        if current_pos in remaining_targets:
                            cleaned.add(current_pos)
                            remaining_targets.discard(current_pos)
                            # 一旦找到新目标，立即切换回贪心
                            print(f"Found new target {current_pos} during random walk, switching back to greedy.")
                            break
        else:
            # 未陷入僵局，使用贪心算法找最近目标
            nearest_target = None
            min_path = None
            for target in remaining_targets:
                path = find_shortest_path_with_direction(current_pos, current_dir, target, map_grid)
                if path is not None and (min_path is None or len(path) < len(min_path)):
                    min_path = path
                    nearest_target = target

            if not min_path: # 理论上不应发生
                break

            # 执行到最近目标的路径
            plan += min_path
            # 更新状态
            for instr in min_path:
                if instr == 'L': current_dir = TURN_LEFT[current_dir]
                elif instr == 'R': current_dir = TURN_RIGHT[current_dir]
                elif instr == 'M':
                    new_pos = move_with_wrap(current_pos, current_dir, map_grid)
                    if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                        current_pos = new_pos
                        if map_grid[current_pos[1]][current_pos[0]] == ' ':
                            cleaned.add(current_pos)
                            remaining_targets.discard(current_pos)
    return plan


def get_final_state_after_plan(start_pos, start_dir, plan, map_grid):
    """模拟执行 plan 后的最终状态"""
    current_pos = start_pos
    current_dir = start_dir
    for instruction in plan:
        if instruction == 'L':
            current_dir = TURN_LEFT[current_dir]
        elif instruction == 'R':
            current_dir = TURN_RIGHT[current_dir]
        elif instruction == 'M':
            new_pos = move_with_wrap(current_pos, current_dir, map_grid)
            if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                current_pos = new_pos
    return current_pos, current_dir

def generate_patch_plan_random_walk(from_pos, from_dir, map_grid, steps=20):
    """简单的随机游走补丁生成器"""
    plan = ""
    current_pos = from_pos
    current_dir = from_dir
    for _ in range(steps):
        action = random.choice(['L', 'R', 'M'])
        plan += action
        if action == 'L': current_dir = TURN_LEFT[current_dir]
        elif action == 'R': current_dir = TURN_RIGHT[current_dir]
        elif action == 'M':
            new_pos = move_with_wrap(current_pos, current_dir, map_grid)
            if not utils.is_wall(map_grid, new_pos[0], new_pos[1]):
                current_pos = new_pos
    return plan