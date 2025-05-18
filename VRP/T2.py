import math
import heapq
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


class DronePathPlanner:
    def __init__(self, drones, targets, obstacles=None):
        self.drones = drones  # 无人机初始位置和状态
        self.targets = targets  # 目标点字典 {名称: (x,y)}
        self.obstacles = obstacles if obstacles else []  # 障碍物列表 [(x,y,radius)]

        # 记录路径和分配方案
        self.paths = {drone_id: [pos] for drone_id, pos in drones.items()}
        self.allocations = {drone_id: [] for drone_id in drones}

    def euclidean_distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def a_star(self, start, goal, drone_id, current_time):
        """A*算法实现避障路径规划"""

        def heuristic(pos):
            return self.euclidean_distance(pos, goal)

        # 定义网格大小和分辨率
        grid_size = 2000  # 整个区域大小
        resolution = 50  # 网格分辨率

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start), 0, start, [start]))
        closed_set = set()

        while open_set:
            _, g, current, path = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)

            # 检查是否到达目标
            if self.euclidean_distance(current, goal) < resolution:
                return path + [goal]

            # 生成邻近节点
            for dx in [-resolution, 0, resolution]:
                for dy in [-resolution, 0, resolution]:
                    if dx == 0 and dy == 0:
                        continue

                    neighbor = (current[0] + dx, current[1] + dy)

                    # 检查边界
                    if not (
                        0 <= neighbor[0] <= grid_size and 0 <= neighbor[1] <= grid_size
                    ):
                        continue

                    # 检查障碍物碰撞
                    collision = False
                    for ox, oy, radius in self.obstacles:
                        if (
                            self.euclidean_distance(neighbor, (ox, oy)) < radius + 50
                        ):  # 50m安全距离
                            collision = True
                            break
                    if collision:
                        continue

                    # 检查与其他无人机的安全距离
                    safe = True
                    for other_drone, other_pos in self.drones.items():
                        if other_drone != drone_id:
                            dist = self.euclidean_distance(neighbor, other_pos)
                            if dist < 50:  # 最小间距约束
                                safe = False
                                break
                    if not safe:
                        continue

                    new_g = g + self.euclidean_distance(current, neighbor)
                    heapq.heappush(
                        open_set,
                        (
                            new_g + heuristic(neighbor),
                            new_g,
                            neighbor,
                            path + [neighbor],
                        ),
                    )

        return None  # 没有找到路径

    def assign_targets(self):
        """初始目标分配（简单最近邻方法）"""
        unassigned = set(self.targets.keys())

        while unassigned:
            for drone_id, pos in self.drones.items():
                if not unassigned:
                    break

                # 找到最近未分配目标
                nearest = min(
                    unassigned,
                    key=lambda t: self.euclidean_distance(pos, self.targets[t]),
                )
                self.allocations[drone_id].append(nearest)
                unassigned.remove(nearest)

                # 更新无人机位置到目标点
                self.drones[drone_id] = self.targets[nearest]
                self.paths[drone_id].append(self.targets[nearest])

    def dynamic_replan(self, new_target, new_obstacle, current_time):
        """动态重规划处理新目标和障碍"""
        # 添加新障碍
        self.obstacles.append(new_obstacle)

        # 添加新目标
        new_target_name = f"T{len(self.targets)+1}"
        self.targets[new_target_name] = new_target

        # 找到最适合处理新目标的无人机（最近且满足约束）
        best_drone = None
        min_dist = float("inf")

        for drone_id, pos in self.drones.items():
            dist = self.euclidean_distance(pos, new_target)
            if dist < min_dist:
                # 检查路径是否可行
                path = self.a_star(pos, new_target, drone_id, current_time)
                if path:
                    min_dist = dist
                    best_drone = drone_id
                    best_path = path

        if best_drone:
            # 分配新目标给最佳无人机
            self.allocations[best_drone].append(new_target_name)

            # 更新路径
            self.paths[best_drone].extend(best_path[1:])  # 跳过第一个点（当前位置）
            self.drones[best_drone] = new_target

            # 重新规划该无人机的返航路径（如果需要）
            return_path = self.a_star(new_target, (0, 0), best_drone, current_time)
            if return_path:
                self.paths[best_drone].extend(return_path[1:])

        # 对其他无人机检查是否需要避障
        for drone_id, pos in self.drones.items():
            if drone_id == best_drone:
                continue

            current_path = self.paths[drone_id]
            if len(current_path) > 1:
                next_point = current_path[-1]  # 假设无人机正在前往的下一个点

                # 检查路径是否穿过障碍
                needs_replan = False
                for ox, oy, radius in self.obstacles:
                    # 简单线段与圆相交检测
                    if self.line_circle_intersection(pos, next_point, (ox, oy), radius):
                        needs_replan = True
                        break

                if needs_replan:
                    new_path = self.a_star(pos, next_point, drone_id, current_time)
                    if new_path:
                        self.paths[drone_id] = current_path[:-1] + new_path[1:]

    def line_circle_intersection(self, p1, p2, center, radius):
        """检查线段是否与圆相交"""
        # 线段参数方程: p = p1 + t*(p2-p1), t∈[0,1]
        # 圆心到线段的距离
        x1, y1 = p1
        x2, y2 = p2
        cx, cy = center

        # 向量化计算
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy

        # 线段是点的情况
        if l2 == 0:
            return self.euclidean_distance(p1, center) <= radius

        # 计算投影参数t
        t = ((cx - x1) * dx + (cy - y1) * dy) / l2
        t = max(0, min(1, t))

        # 投影点
        projection = (x1 + t * dx, y1 + t * dy)

        # 检查距离
        return self.euclidean_distance(projection, center) <= radius

    def visualize(self):
        """可视化结果，风格与T1一致"""
        plt.figure(figsize=(10, 8))
        # 绘制所有点
        for name, coord in self.targets.items():
            if name == "T1":  # 只为第一个目标点加label，防止重复
                plt.plot(coord[0], coord[1], "bo", markersize=8, label=name)
            else:
                plt.plot(coord[0], coord[1], "bo", markersize=8)
            plt.text(coord[0] + 20, coord[1] + 20, name, fontsize=12)
        plt.plot(0, 0, "ro", markersize=10, label="Depot")
        # 绘制障碍物
        for x, y, r in self.obstacles:
            circle = Circle((x, y), r, color="gray", alpha=0.5)
            plt.gca().add_patch(circle)
        # 绘制无人机路径
        colors = ["g", "m", "c", "y", "k"]
        for i, (drone_id, path) in enumerate(self.paths.items()):
            if len(path) > 1:
                x_coords = [p[0] for p in path]
                y_coords = [p[1] for p in path]
                plt.plot(
                    x_coords,
                    y_coords,
                    colors[i % len(colors)],
                    linestyle="-",
                    linewidth=2,
                    label=f"UAV {i+1}: {sum(self.euclidean_distance(path[j], path[j+1]) for j in range(len(path)-1)):.1f}m",
                )
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("UAV Path Planning ")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    def animate(self, interval=500):
        """动态展示无人机路径规划过程（与T1风格一致）"""
        import matplotlib.animation as animation

        fig, ax = plt.subplots(figsize=(10, 8))
        # 绘制所有点
        for name, coord in self.targets.items():
            if name == "T1":
                ax.plot(coord[0], coord[1], "bo", markersize=8, label=name)
            else:
                ax.plot(coord[0], coord[1], "bo", markersize=8)
            ax.text(coord[0] + 20, coord[1] + 20, name, fontsize=12)
        ax.plot(0, 0, "ro", markersize=10, label="Depot")
        # 绘制障碍物
        for x, y, r in self.obstacles:
            circle = Circle((x, y), r, color="gray", alpha=0.5)
            ax.add_patch(circle)
        colors = ["g", "m", "c", "y", "k"]
        max_len = max(len(path) for path in self.paths.values())
        lines = []
        points = []
        for i, (drone_id, path) in enumerate(self.paths.items()):
            (line,) = ax.plot(
                [],
                [],
                colors[i % len(colors)],
                linestyle="-",
                linewidth=2,
                label=f"UAV {i+1}",
            )
            (point,) = ax.plot(
                [], [], marker="o", color=colors[i % len(colors)], markersize=10
            )
            lines.append(line)
            points.append(point)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("UAV Path Planning (Animation)")
        ax.legend()
        ax.grid(True)
        ax.axis("equal")

        def update(frame):
            for i, (drone_id, path) in enumerate(self.paths.items()):
                if frame < len(path):
                    x = [p[0] for p in path[: frame + 1]]
                    y = [p[1] for p in path[: frame + 1]]
                    lines[i].set_data(x, y)
                    points[i].set_data([x[-1]], [y[-1]])  # 修正：必须传序列
                else:
                    x = [p[0] for p in path]
                    y = [p[1] for p in path]
                    lines[i].set_data(x, y)
                    points[i].set_data([x[-1]], [y[-1]])  # 修正：必须传序列
            return lines + points

        ani = animation.FuncAnimation(
            fig, update, frames=max_len, interval=interval, blit=True, repeat=False
        )
        plt.show()


# 问题2的实例数据
initial_drones = {"U1": (0, 0), "U2": (0, 0), "U3": (0, 0)}

targets = {
    "T1": (1200, 800),
    "T2": (300, 450),
    "T3": (950, 200),
    "T4": (600, 1200),
    "T5": (1500, 500),
}

# 创建路径规划器
planner = DronePathPlanner(initial_drones, targets)

# 初始目标分配
planner.assign_targets()

# 模拟在100s时新增障碍和紧急目标
new_obstacle = (900, 250, 100)  # (x, y, radius)
new_target = (800, 600)
current_time = 100  # 假设在100s时发生动态变化

# 动态重规划
planner.dynamic_replan(new_target, new_obstacle, current_time)

# 输出结果
print("调整后的路径规划:")
for drone_id, path in planner.paths.items():
    print(f"{drone_id}路径:", " → ".join([f"({x},{y})" for x, y in path]))
    print(
        f"总飞行距离: {sum(planner.euclidean_distance(path[i], path[i+1]) for i in range(len(path)-1)):.1f}m"
    )
    print(f"分配的目标点: {planner.allocations[drone_id]}")
    print()

# 计算最短完成时间
flight_times = []
for drone_id, path in planner.paths.items():
    distance = sum(
        planner.euclidean_distance(path[i], path[i + 1]) for i in range(len(path) - 1)
    )
    time = distance / 50  # 假设最大速度50m/s
    flight_times.append(time)

shortest_time = max(flight_times)
print(f"T1-T6全覆盖的最短时间: {shortest_time:.1f}s")

# 可视化
planner.visualize()

# 动态可视化
planner.animate(interval=500)
