import sys

for file in sys.argv[1:]:
    with open(file, 'r') as f:
        content = f.readlines()[1:]

    positions = []

    for y, line in enumerate(content):
        for x, c in enumerate(line):
            if c == 'P':
                positions.append((x, y))

    if len(positions) == 0:
        continue

    assert len(positions) == 2

    manhattan_dist = abs(positions[0][0] - positions[1][0]) + abs(positions[0][1] - positions[1][1])
    if manhattan_dist == 1:
        print(f"File: {file}")
