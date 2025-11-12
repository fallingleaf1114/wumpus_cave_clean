# utils.py

def load_problem_file(filepath):
    """
    Loads a problem file and parses its content.

    Args:
        filepath (str): Path to the problem file.

    Returns:
        dict: A dictionary containing the problem type ('CHECK_PLAN' or 'FIND_PLAN'),
              the plan string (if CHECK_PLAN), and the map representation.
              {
                  'type': str,
                  'plan': str or None,
                  'map': list[list[str]], # 2D list of characters
                  'height': int,
                  'width': int,
                  'start_pos': (int, int) or None, # (x, y)
                  'start_dirs': list[str] or None # e.g., ['>'], ['^', '>', 'v', '<'], or [None]
              }
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Remove trailing newlines and any empty lines at the end
    lines = [line.rstrip('\n') for line in lines if line.rstrip('\n')]
    if not lines:
        raise ValueError(f"Problem file {filepath} is empty or only contains whitespace.")

    # Determine problem type
    first_line = lines[0].strip()
    if first_line == "CHECK PLAN":
        problem_type = "CHECK_PLAN"
        plan_line = lines[1].strip()
        map_start_idx = 2
    elif first_line == "FIND PLAN":
        problem_type = "FIND_PLAN"
        plan_line = None
        map_start_idx = 1
    else:
        raise ValueError(f"Invalid problem type on first line of {filepath}: {first_line}")

    # Parse map
    map_lines = lines[map_start_idx:]
    height = len(map_lines)
    width = len(map_lines[0]) if height > 0 else 0 # Assuming rectangular map

    map_grid = [list(row) for row in map_lines]

    # Find start position and initial direction(s)
    start_pos = None
    start_dirs = []
    for y, row in enumerate(map_grid):
        for x, char in enumerate(row):
            if char in ['S', '^', '>', 'v', '<']:
                if start_pos is not None:
                    raise ValueError(f"Multiple start positions found in map {filepath}")
                start_pos = (x, y)
                if char == 'S':
                    start_dirs = [None] # Unknown direction
                elif char == '^':
                    start_dirs = ['^']
                elif char == '>':
                    start_dirs = ['>']
                elif char == 'v':
                    start_dirs = ['v']
                elif char == '<':
                    start_dirs = ['<']

    if start_pos is None:
        raise ValueError(f"No start position found in map {filepath}")

    # If initial direction was unknown (S), set potential_dirs to all four
    if start_dirs == [None]:
        start_dirs = ['^', '>', 'v', '<']

    return {
        'type': problem_type,
        'plan': plan_line,
        'map': map_grid,
        'height': height,
        'width': width,
        'start_pos': start_pos,
        'start_dirs': start_dirs
    }

def is_wall(map_grid, x, y):
    """Checks if the given coordinates are a wall."""
    # Wrap around for boundary check (not for wall check)
    # height = len(map_grid)
    # width = len(map_grid[0]) if height > 0 else 0
    # x = x % width
    # y = y % height
    if 0 <= y < len(map_grid) and 0 <= x < len(map_grid[0]):
        return map_grid[y][x] == 'X'
    return False # Coordinates outside the map are not walls, but this function is for internal check

def is_free_space(map_grid, x, y):
    """Checks if the given coordinates are a free space (not wall or start)."""
    if 0 <= y < len(map_grid) and 0 <= x < len(map_grid[0]):
        char = map_grid[y][x]
        return char in [' ', 'S', '^', '>', 'v', '<'] # Consider start chars as free *initially*, but only ' ' is a *cleanable* free space
    return False

def get_starting_positions_and_directions(map_grid):
    """Finds the starting position and possible initial directions."""
    for y, row in enumerate(map_grid):
        for x, char in enumerate(row):
            if char in ['S', '^', '>', 'v', '<']:
                if char == 'S':
                    return (x, y), ['^', '>', 'v', '<']
                else:
                    return (x, y), [char]
    return None, None

# Example usage (for testing the parser):
# problem_data = load_problem_file("problems/problem_a_00.txt")
# print("Type:", problem_data['type'])
# print("Plan:", problem_data['plan'])
# print("Map:")
# for row in problem_data['map']:
#     print(''.join(row))
# print("Height:", problem_data['height'])
# print("Width:", problem_data['width'])
# print("Start Pos:", problem_data['start_pos'])
# print("Start Dirs:", problem_data['start_dirs'])
