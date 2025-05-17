import math
import heapq
import matplotlib.pyplot as plt

# 起点
start_pos = (0, 0)

# 目标点坐标
targets = {
    "T1": (1200, 800),
    "T2": (300, 450),
    "T3": (950, 200),
    "T4": (600, 1200),
    "T5": (1500, 500),
    "T6": (800, 600)
}

# 每架无人机分配的目标点
assignments = {
    "U1": ["T2", "T4"],
    "U2": ["T3", "T6"],  # 避障路径
    "U3": ["T1", "T5"]
}

# 圆形障碍区域
obstacle_center = (900, 250)
obstacle_radius = 100

# 欧几里得距离
def euclidean(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

# 判断是否在障碍区内
def in_obstacle(pos, center=obstacle_center, radius=obstacle_radius):
    return euclidean(pos, center) < radius

# A*避障算法
def a_star(start, goal, grid_size=20, bounds=(0, 1600, 0, 1300)):
    x_min, x_max, y_min, y_max = bounds
    open_set = []
    heapq.heappush(open_set, (euclidean(start, goal), 0, start, [start]))
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if euclidean(current, goal) <= grid_size:
            path.append(goal)
            return path

        if current in visited:
            continue
        visited.add(current)

        x, y = current
        for dx in [-grid_size, 0, grid_size]:
            for dy in [-grid_size, 0, grid_size]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (x + dx, y + dy)
                if (x_min <= neighbor[0] <= x_max and
                    y_min <= neighbor[1] <= y_max and
                    not in_obstacle(neighbor)):
                    heapq.heappush(open_set, (
                        cost + euclidean(neighbor, goal),
                        cost + euclidean(neighbor, goal),
                        neighbor,
                        path + [neighbor]
                    ))
    return None

# 绘图
def plot_paths():
    colors = {"U1": "blue", "U2": "green", "U3": "orange"}

    plt.figure(figsize=(10, 8))

    # 绘制目标点
    for name, coord in targets.items():
        plt.plot(*coord, 'ko')
        plt.text(coord[0] + 10, coord[1] + 10, name, fontsize=9)

    # 绘制障碍圆
    circle = plt.Circle(obstacle_center, obstacle_radius, color='r', alpha=0.3, label="Obstacle")
    plt.gca().add_patch(circle)

    # 起点
    plt.plot(*start_pos, 'ks', label="Start")

    # 绘制路径
    for drone, point_names in assignments.items():
        pos = start_pos
        path_x = [pos[0]]
        path_y = [pos[1]]
        for pt_name in point_names:
            pt = targets[pt_name]
            if drone == "U2":  # U2使用A*避障
                segment = a_star(pos, pt)
            else:
                segment = [pos, pt]
            if segment:
                xs, ys = zip(*segment)
                path_x.extend(xs[1:])  # 避免重复首点
                path_y.extend(ys[1:])
                pos = pt
        plt.plot(path_x, path_y, '--', label=f"{drone} Path", color=colors[drone])

    plt.legend()
    plt.title("3-Drone Path Planning with A* Obstacle Avoidance for U2")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

# 执行绘图
if __name__ == "__main__":
    plot_paths()