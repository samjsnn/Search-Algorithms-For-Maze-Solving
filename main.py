from collections import deque
from queue import PriorityQueue
import math
import heapq


# Define the dictionary to map coordinates to directions
directions_map = {
    (0, 1): "R",
    (0, -1): "L",
    (1, 0): "D",
    (-1, 0): "U",
    (1, 1): "DR",
    (-1, -1): "UL",
    (-1, 1): "UR",
    (1, -1): "DL"
}

# Define a function that solves the path from the starting point to the goal on the board.
# Function takes the board as input and returns a tuple containing the path and the number of states explored.


def solve_path_BFS(board):
    row, col = len(board), len(board[0])
    # find starting point
    start = next(((r, c) for r, row in enumerate(board)
                 for c, val in enumerate(row) if val == 'S'), None)
    if not start:
        return None
    # initializing the queue with the starting point and a path of length 0
    queue = deque()
    queue.appendleft((start[0], start[1], 0, []))
    # initializing the set of visited coordinates and the number of states explored to 0
    visited = set()
    states = 0

    while len(queue) != 0:
        # get next coord from queue
        coord = queue.pop()
        if coord[:2] in visited:
            continue
        # if coord has been visited skip it
        visited.add(coord[:2])
        # if coord is not a wall, increment the number of states explored
        if board[coord[0]][coord[1]] != "X":
            states += 1
        # if the coord is the goal, return the current path and the number of states explored
        if board[coord[0]][coord[1]] == "G":
            return coord, states
        # generate a list of valid directions to move in
        directions = []
        for dr in [-1, 1, 0]:
            for dc in [-1, 1, 0]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = coord[0] + dr, coord[1] + dc
                if nr < 0 or nr >= row or nc < 0 or nc >= col or board[nr][nc] == "X":
                    continue
                if dr == 0 or dc == 0:
                    directions.append([dr, dc,])
                else:
                    if board[coord[0]][nc] == "X" or board[nr][coord[1]] == "X":
                        continue
                    directions.insert(0, [dr, dc])
        # add the new coordinates and path to the queue for each valid direction
        for dir in directions:
            nr, nc = coord[0]+dir[0], coord[1]+dir[1]
            if (nr, nc) in visited:
                continue
            new_path = coord[3] + [dir]
            if dir[0] == 0 or dir[1] == 0:
                cost = 2
            else:
                cost = 1
            queue.appendleft((nr, nc, coord[2]+cost, new_path))

    return None, states


def solve_path_UCS(board):
    row, col = len(board), len(board[0])
    # find starting point
    start = next(((r, c) for r, row in enumerate(board)
                 for c, val in enumerate(row) if val == 'S'), None)
    if not start:
        return None
    # initializing the priority queue with the starting point and a path cost of 0
    queue = PriorityQueue()
    queue.put((0, start[0], start[1], 0, []))
    # initializing the set of visited coordinates and the number of states explored to 0
    visited = set()
    states = 0

    while not queue.empty():
        # get next coord from priority queue
        cost, coord_r, coord_c, depth, path = queue.get()
        if (coord_r, coord_c) in visited:
            continue
        # if coord has been visited skip it
        visited.add((coord_r, coord_c))
        # if coord is not a wall, increment the number of states explored
        if board[coord_r][coord_c] != "X":
            states += 1
        # if the coord is the goal, return the current path and the number of states explored
        if board[coord_r][coord_c] == "G":
            return (coord_r, coord_c, depth, path), states
        # generate a list of valid directions to move in
        directions = []
        for dr in [-1, 1, 0]:
            for dc in [-1, 1, 0]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = coord_r + dr, coord_c + dc
                if nr < 0 or nr >= row or nc < 0 or nc >= col or board[nr][nc] == "X":
                    continue
                if dr == 0 or dc == 0:
                    directions.append([dr, dc, 2])
                else:
                    if board[coord_r][nc] == "X" or board[nr][coord_c] == "X":
                        continue
                    directions.insert(0, [dr, dc, 1])
        # add the new coordinates, path cost, and path to the priority queue for each valid direction
        for dir in directions:
            nr, nc = coord_r + dir[0], coord_c + dir[1]
            if (nr, nc) in visited:
                continue
            new_path = path + [dir]
            queue.put((depth + dir[2], nr, nc, depth + dir[2], new_path))

    return None, states


def solve_path_ID(board):
    row, col = len(board), len(board[0])
    # find starting point
    start = next(((r, c) for r, row in enumerate(board)
                 for c, val in enumerate(row) if val == 'S'), None)
    if not start:
        return None
    # initializing the stack with the starting point and a path of length 0
    stack = {start: ([], 0)}
    # initializing the number of states explored to 0
    states = 0

    for depth in range(1, row*col+1):  # iterative deepening
        new_stack = {}
        for coord, (path, cost) in stack.items():
            # if coord is not a wall, increment the number of states explored
            if board[coord[0]][coord[1]] != "X":
                states += 1
            # if the coord is the goal, return the current path and the number of states explored
            if board[coord[0]][coord[1]] == "G":
                return (path, cost), states
            # generate a list of valid directions to move in
            directions = []
            for dr in [-1, 1, 0]:
                for dc in [-1, 1, 0]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = coord[0] + dr, coord[1] + dc
                    if nr < 0 or nr >= row or nc < 0 or nc >= col or board[nr][nc] == "X":
                        continue
                    if dr == 0 or dc == 0:
                        directions.append([dr, dc,])
                    else:
                        if board[coord[0]][nc] == "X" or board[nr][coord[1]] == "X":
                            continue
                        directions.insert(0, [dr, dc])
            # add the new coordinates and path to the new stack for each valid direction
            for dir in directions:
                nr, nc = coord[0]+dir[0], coord[1]+dir[1]
                new_path = path + [dir]
                if dir[0] == 0 or dir[1] == 0:
                    move_cost = 2
                else:
                    move_cost = 1
                new_cost = cost + move_cost
                if (nr, nc) not in new_stack or new_cost < new_stack[(nr, nc)][1]:
                    new_stack[(nr, nc)] = (new_path, new_cost)

        stack = new_stack

    return None, states


# Define the A* search function


def a_star_search(board):
    row, col = len(board), len(board[0])
    # find starting point
    start = next(((r, c) for r, row in enumerate(board)
                 for c, val in enumerate(row) if val == 'S'), None)
    if not start:
        return None

    # Define the heuristic function
    # def heuristic(coord):
    #     # Calculate the Manhattan distance between the current coord and the goal
    #     goal = next(((r, c) for r, row in enumerate(board)
    #                  for c, val in enumerate(row) if val == 'G'), None)
    #     return abs(coord[0] - goal[0]) + abs(coord[1] - goal[1])

    # # Initialize the priority queue with the starting point and a path of length 0
    # queue = [(heuristic(start), start[0], start[1], 0, [])]
    # # Initialize the set of visited coordinates and the number of states explored to 0
    # visited = set()
    # states = 0

    def heuristic(coord):
        # Calculate the Euclidean distance between the current coord and the goal
        goal = next(((r, c) for r, row in enumerate(board)
                     for c, val in enumerate(row) if val == 'G'), None)
        if not goal:
            return 0
        return math.sqrt((coord[0] - goal[0]) ** 2 + (coord[1] - goal[1]) ** 2)

    # Initialize the priority queue with the starting point and a path of length 0
    queue = [(heuristic(start), start[0], start[1], 0, [])]
    # Initialize the set of visited coordinates and the number of states explored to 0
    visited = set()
    states = 0

    while len(queue) != 0:
        # Get the next coord from the priority queue
        _, r, c, cost, path = heapq.heappop(queue)
        coord = (r, c)
        if coord in visited:
            continue
        # If coord has been visited, skip it
        visited.add(coord)
        # If coord is not a wall, increment the number of states explored
        if board[r][c] != "X":
            states += 1
        # If the coord is the goal, return the current path and the number of states explored
        if board[r][c] == "G":
            return (r, c, cost, path), states
        # Generate a list of valid directions to move in
        directions = []
        for dr in [-1, 1, 0]:
            for dc in [-1, 1, 0]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if nr < 0 or nr >= row or nc < 0 or nc >= col or board[nr][nc] == "X":
                    continue
                if dr == 0 or dc == 0:
                    directions.append([dr, dc, ])
                else:
                    if board[r][nc] == "X" or board[nr][c] == "X":
                        continue
                    directions.insert(0, [dr, dc])
        # Add the new coordinates and path to the priority queue for each valid direction
        for dir in directions:
            nr, nc = r + dir[0], c + dir[1]
            if (nr, nc) in visited:
                continue
            new_path = path + [dir]
            if dir[0] == 0 or dir[1] == 0:
                step_cost = 2
            else:
                step_cost = 1
            heapq.heappush(queue, (cost + step_cost +
                           heuristic((nr, nc)), nr, nc, cost + step_cost, new_path))

    # If the goal node is not found, return None
    return None, states


def get_neighbors(path, i):
    neighbors = []
    if i > 0:
        neighbors.append(i - 1)
    if i < len(path) - 1:
        neighbors.append(i + 1)
    return neighbors


def is_valid_path(board, start, path):
    row, col = len(board), len(board[0])
    current_pos = start
    for dir in path:
        current_pos = (current_pos[0] + dir[0], current_pos[1] + dir[1])
        if current_pos[0] < 0 or current_pos[0] >= row or current_pos[1] < 0 or current_pos[1] >= col:
            return False
        if board[current_pos[0]][current_pos[1]] == 'X':
            return False
        if board[current_pos[0]][current_pos[1]] == 'G':
            return True
    return False


def solve_path_hill_climb(board, M=10):
    row, col = len(board), len(board[0])
    # find starting point
    start = next(((r, c) for r, row in enumerate(board)
                 for c, val in enumerate(row) if val == 'S'), None)
    if not start:
        return None
    # initializing the starting path with M directions
    current_path = [[1, 0] if i % 2 == 0 else [0, 1] for i in range(M)]
    # compute the cost of the starting path
    current_cost = sum(2 if dir[0] == 0 or dir[1] ==
                       0 else 1 for dir in current_path)
    # initializing the set of visited coordinates and the number of states explored to 0
    visited = set()
    states = 0

    while not is_valid_path(board, start, current_path):
        # check if current position is G state
        if board[start[0]][start[1]] == 'G':
            return current_path, states
        # generate a list of positions to swap with the current path
        swap_positions = []
        for i in range(M):
            neighbors = get_neighbors(current_path, i)
            for j in neighbors:
                swap_positions.append((i, j))
        # if no valid position found, return None
        if not swap_positions:
            return None, states
        # select the position that reduces the cost the most
        min_cost = current_cost
        min_path = None
        for pos in swap_positions:
            i, j = pos
            new_path = current_path.copy()
            new_path[i], new_path[j] = new_path[j], new_path[i]
            new_cost = sum(2 if dir[0] == 0 or dir[1] ==
                           0 else 1 for dir in new_path)
            if new_cost < min_cost:
                min_cost = new_cost
                min_path = new_path
        # if no position reduces the cost, return the current path
        if min_path is None:
            return current_path, states
        # update the current path and cost
        current_path = min_path
        current_cost = min_cost
        # update the starting point and the set of visited coordinates
        start = (start[0]+current_path[0][0], start[1]+current_path[0][1])
        visited.add(start)
        # increment the number of states explored
        states += 1
        # if the current path is valid, return it
    return None, states


# read from file
with open("input.txt") as f:
    # keeps track of which algorithm to use
    algorithm = int(f.readline().strip('\n'))
    n = int(f.readline().strip('\n'))
    # initialize empty board to store game board
    board = []
    # iterates over rest of lines in file and appends rows
    for line in f:
        board.append([i for i in line.strip("\n")])
    if algorithm == 1:
        path, states = solve_path_BFS(board)
        # write to output file
        if path is None:
            with open("output.txt", "w") as f:
                f.write("No path was found")
        else:
            # create copy of board to track and display which path taken
            map_copy = [row.copy() for row in board]
            start = (0, 0)
            for r in range(len(map_copy)):
                for c in range(len(map_copy[0])):
                    if map_copy[r][c] == 'S':
                        start = (r, c)
                        break
                else:
                    continue
                break
            # iterate over each move in solution path and updates tuple
            for move in path[3]:
                row, col = start = (start[0] + move[0], start[1] + move[1])
                map_copy[row][col] = 'o'

            with open("output.txt", "w") as f:
                f.write('-'.join([directions_map[tuple(move)]

                        for move in path[3]]))
                f.write("\n")
                f.write('\n')
                path_length = len(path[3])
                path_cost = sum(1 if move[0] != 0 and move[1]
                                != 0 else 2 for move in path[3])
                f.write(
                    f"Path length: {path_length}\nPath cost: {path_cost}\n")
                f.write(f"States Explored: {states}\n")
                f.write("\n")
                f.write('\n'.join([''.join(row) for row in map_copy]))
    elif algorithm == 2:
        path, states = solve_path_UCS(board)

    # write to output file
        if path is None:
            with open("output.txt", "w") as f:
                f.write("No path was found")
        else:
            # create copy of board to track and display which path taken
            map_copy = [row.copy() for row in board]
            start = (0, 0)
            for r in range(len(map_copy)):
                for c in range(len(map_copy[0])):
                    if map_copy[r][c] == 'S':
                        start = (r, c)
                        break
                else:
                    continue
                break
            # iterate over each move in solution path and updates tuple
            for move in path[3]:
                row, col = start = (start[0] + move[0], start[1] + move[1])
                map_copy[row][col] = 'o'
            print(path)

            with open("output.txt", "w") as f:
                f.write('-'.join([directions_map[tuple(move[:2])]
                        for move in path[3]]))
                f.write("\n")
                f.write('\n')
                path_length = len(path[3])
                path_cost = sum(
                    1 if move[0] != 0 and move[1] != 0 else 2 for move in path[3])
                f.write(
                    f"Path length: {path_length}\nPath cost: {path_cost}\n")
                f.write(f"States Explored: {states}\n")
                f.write("\n")
                f.write('\n'.join([''.join(row) for row in map_copy]))
    elif algorithm == 3:
        path, states = solve_path_ID(board)
        if path is None:
            with open("output.txt", "w") as f:
                f.write("No path was found")
        else:
            # create copy of board to track and display which path taken
            map_copy = [row.copy() for row in board]
            start = (0, 0)
            for r in range(len(map_copy)):
                for c in range(len(map_copy[0])):
                    if map_copy[r][c] == 'S':
                        start = (r, c)
                        break
                else:
                    continue
                break
            # iterate over each move in solution path and updates tuple
            for move in path[0]:
                row, col = start = (start[0] + move[0], start[1] + move[1])
                map_copy[row][col] = 'o'

            with open("output.txt", "w") as f:
                f.write('-'.join([directions_map[tuple(move)]
                        for move in path[0]]))
                f.write("\n")
                f.write('\n')
                path_length = len(path[0])
                path_cost = sum(
                    1 if move[0] != 0 and move[1] != 0 else 2 for move in path[0])
                f.write(
                    f"Path length: {path_length}\nPath cost: {path_cost}\n")
                f.write(f"States Explored: {states}\n")
                f.write("\n")
                f.write('\n'.join([''.join(row) for row in map_copy]))
    elif algorithm == 4:
        path, states = a_star_search(board)

        # Write to output file
        if path is None:
            with open("output.txt", "w") as f:
                f.write("No path was found")
        else:
            # Create a copy of the board to track and display which path was taken
            map_copy = [row.copy() for row in board]
            start = (0, 0)
            for r in range(len(map_copy)):
                for c in range(len(map_copy[0])):
                    if map_copy[r][c] == 'S':
                        start = (r, c)
                        break
                else:
                    continue
                break
            # Update the map copy with the moves taken
            for move in path[3]:
                row, col = start = (start[0] + move[0], start[1] + move[1])
                map_copy[row][col] = 'o'

            with open("output.txt", "w") as f:
                f.write('-'.join([directions_map[tuple(move)]
                        for move in path[3]]))
                f.write("\n")
                f.write('\n')
                path_length = len(path[3])
                path_cost = sum(1 if move[0] != 0 and move[1]
                                != 0 else 2 for move in path[3])
                # f.write(f"Goal State: {path[0]}, {path[1]}\n")
                f.write(f"Path Length: {path_length}\n")
                f.write(f"Path Cost: {path_cost}\n")
                f.write(f"States Explored: {states}\n")
                f.write("\n")
                f.write('\n'.join([''.join(row) for row in map_copy]))
    else:
        path, steps = solve_path_hill_climb(board)
        print(path)
        print(steps)
        if path is None:
            with open("output.txt", "w") as f:
                f.write("No path was found")
        else:
            # create copy of board to track and display which path taken
            map_copy = [row.copy() for row in board]
            start = (0, 0)
            for r in range(len(map_copy)):
                for c in range(len(map_copy[0])):
                    if map_copy[r][c] == 'S':
                        start = (r, c)
                        break
                else:
                    continue
                break
            # iterate over each move in solution path and updates tuple
            for move in path:
                row, col = start = (start[0] + move[0], start[1] + move[1])
                map_copy[row][col] = 'o'

            with open("output.txt", "w") as f:
                f.write('-'.join([directions_map[tuple(move)]
                                  for move in path]))
                f.write("\n")
                f.write('\n')
                path_length = len(path)
                path_cost = sum(1 if move[0] != 0 and move[1]
                                != 0 else 2 for move in path)
                f.write(
                    f"Path length: {path_length}\nPath cost: {path_cost}\n")
                f.write(f"Nodes expanded: {steps}\n")
                f.write("\n")
                f.write('\n'.join([''.join(row) for row in map_copy]))

# finished except hill climb function not working or complete
