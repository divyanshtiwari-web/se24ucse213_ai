import heapq
import random

# ---------- GRID GENERATION ----------
def generate_grid(size, density):
    grid = [[0 for _ in range(size)] for _ in range(size)]

    for i in range(size):
        for j in range(size):
            if random.random() < density:
                grid[i][j] = 1

    return grid


# ---------- HEURISTIC ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ---------- A* ----------
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue

                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = current

    return None


# ---------- DYNAMIC PATH ----------
def dynamic_astar(grid, start, goal):
    current = start
    path_taken = [current]

    while current != goal:
        path = astar(grid, current, goal)

        if not path:
            print("No path available!")
            return None

        for step in path[1:]:
            # Random obstacle appears
            if random.random() < 0.2:
                grid[step[0]][step[1]] = 1
                print("Obstacle appeared at:", step)
                break

            current = step
            path_taken.append(current)

            if current == goal:
                print("Reached goal!")
                return path_taken

    return path_taken


# ---------- MAIN ----------
size = 20
grid = generate_grid(size, 0.1)

start = (0, 0)
goal = (19, 19)

grid[start[0]][start[1]] = 0
grid[goal[0]][goal[1]] = 0

path = dynamic_astar(grid, start, goal)

if path:
    print("Final path length:", len(path))
