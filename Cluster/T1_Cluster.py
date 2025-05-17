#!/bin/env python3
import math
import random
from collections import deque
import matplotlib.pyplot as plt
from task import Task
from common import *
from obstacle import Obstacle
from drone import *

plt.rcParams["font.family"] = [
    "PingFang SC",
    "Microsoft YaHei",
]  # 逐个尝试，直到找到存在的字体。出于跨平台考虑。


# 初始化任务
def generate_tasks():
    # 第一题当成普通投放，但载重量为0
    return [
        Task("T1", (1200, 800), "normal", 2, 0),
        Task("T2", (300, 450), "normal", 2, 0),
        Task("T3", (950, 200), "normal", 2, 0),
        Task("T4", (600, 1200), "normal", 2, 0),
        Task("T5", (1500, 500), "normal", 2, 0),
    ]
    # return [
    #     Task("T1", (700, 900), "urgent", 1, 8),
    #     Task("T2", (1200, 800), "normal", 2, 12),
    #     Task("T3", (1000, 1000), "recon", 3, 40),
    #     Task("T4", (300, 300), "normal", 2, 6),
    #     Task("T5", (800, 600), "urgent", 1, 10),
    #     Task("T6", (400, 800), "recon", 3, 30),
    #     Task("T7", (900, 400), "normal", 2, 5),
    #     Task("T8", (1300, 200), "urgent", 1, 7),
    #     Task("T9", (200, 600), "recon", 3, 50),
    #     Task("T10", (600, 1200), "normal", 2, 9),
    # ]


# 简单贪心规划任务
def assign_tasks(
    drones: list[Drone], tasks: list[Task], obstacle: Obstacle | None = None
):
    leader = drones[0]
    leader.is_leader = True
    unassigned = sorted(tasks, key=lambda t: t.priority)

    def drones_too_far_or_close(drones, far_threshold=1000, close_threshold=50):
        n = len(drones)
        for i in range(n):
            for j in range(i + 1, n):
                d1 = drones[i]
                d2 = drones[j]
                # Only check for too close if both drones have taken off (not at BASE)
                d1_taken_off = d1.current_pos != BASE
                d2_taken_off = d2.current_pos != BASE
                dist = math.hypot(
                    d1.current_pos[0] - d2.current_pos[0],
                    d1.current_pos[1] - d2.current_pos[1],
                )
                if abs(dist - far_threshold) < 50:
                    return ("far", d1, d2, dist)
                # Only enforce min distance if both have taken off
                if d1_taken_off and d2_taken_off and dist < close_threshold:
                    return ("close", d1, d2, dist)
        return None

    def circumvent_obstacle(start, end, obstacle):
        # Simple detour: move tangentially to the obstacle for 100m, then toward end
        if not obstacle or not obstacle.is_blocked(start, end):
            return end
        sx, sy = start
        ex, ey = end
        cx, cy = obstacle.center
        # Vector from center to start
        vx, vy = sx - cx, sy - cy
        vlen = math.hypot(vx, vy)
        if vlen == 0:
            # Start is at center, move radially outward
            vx, vy = 1, 0
            vlen = 1
        # Tangent direction (perpendicular)
        tx, ty = -vy / vlen, vx / vlen
        # Move 100m along tangent
        detour_x = sx + tx * 100
        detour_y = sy + ty * 100
        return (detour_x, detour_y)

    breakthrough = True
    while any(not t.assigned for t in unassigned):
        # Check if any two drones are close to 1km apart or too close
        pair_info = drones_too_far_or_close(drones)
        if pair_info:
            status, d1, d2, dist = pair_info
            if status == "far":
                print(f"⚠️ Drones {d1.id} and {d2.id} are close to 1km apart. Moving both closer to leader {leader.id}.")
                for d in [d1, d2]:
                    lx, ly = leader.current_pos
                    dx, dy = d.current_pos
                    vec_x, vec_y = lx - dx, ly - dy
                    d_dist = math.hypot(vec_x, vec_y)
                    if d_dist > 1e-6:
                        move_dist = min(200, d_dist)
                        ratio = move_dist / d_dist
                        target_x = dx + vec_x * ratio
                        target_y = dy + vec_y * ratio
                        target = (target_x, target_y)
                        # Circumvent obstacle if needed
                        if obstacle:
                            target = circumvent_obstacle((dx, dy), target, obstacle)
                        d.move_to(target)
                continue
            elif status == "close":
                print(f"⚠️ Drones {d1.id} and {d2.id} are too close ({dist:.2f}m). Moving them apart.")
                # Move each drone 50m away from the other along the line joining them
                dx1, dy1 = d1.current_pos
                dx2, dy2 = d2.current_pos
                vec_x, vec_y = dx1 - dx2, dy1 - dy2
                vlen = math.hypot(vec_x, vec_y)
                if vlen > 1e-6:
                    move_dist = 50
                    ratio = move_dist / vlen
                    # d1 moves away from d2
                    t1 = (dx1 + vec_x * ratio, dy1 + vec_y * ratio)
                    # d2 moves away from d1
                    t2 = (dx2 - vec_x * ratio, dy2 - vec_y * ratio)
                    # Circumvent obstacle if needed
                    if obstacle:
                        t1 = circumvent_obstacle((dx1, dy1), t1, obstacle)
                        t2 = circumvent_obstacle((dx2, dy2), t2, obstacle)
                    d1.move_to(t1)
                    d2.move_to(t2)
                continue

        if not breakthrough or check_need_return(drones):
            print("⚠️ 电量不足，执行统一返航")
            all_return_to_base(drones)
        breakthrough = False
        for drone in drones:
            # 找最近的未完成任务
            candidates = [t for t in unassigned if not t.assigned]
            candidates = sorted(candidates, key=lambda t: drone.distance(t.coord))
            for task in candidates:
                if task.assigned:
                    continue
                if obstacle and obstacle.is_blocked(drone.current_pos, task.coord):
                    continue
                if task.type == "recon" and drone.hover_time < task.demand:
                    continue
                if (
                    task.type in ["urgent", "normal"]
                    and drone.weight_limit < task.demand
                ):
                    continue
                # 分配任务
                task_time = 0
                if task.type == "recon":
                    task_time = task.demand
                if drone.move_to(task.coord, task_time):
                    task.assigned = True
                    print(f"Drone {drone.id} → {task.id} at {task.coord}")
                    breakthrough = True
                    break

    # 返回基地
    for drone in drones:
        drone.move_to(BASE)


def visualize(drones: list[Drone], tasks: list[Task], obstacle=None):
    colors = ["blue", "green", "red", "purple", "orange"]
    plt.figure(figsize=(10, 8))

    # 绘制任务点
    for task in tasks:
        color = "black"
        marker = "s"
        if task.type == "urgent":
            color = "red"
            marker = "x"
        elif task.type == "normal":
            color = "orange"
        elif task.type == "recon":
            color = "blue"
            marker = "o"
        plt.scatter(*task.coord, color=color, marker=marker, s=100)
        plt.text(task.coord[0] + 10, task.coord[1] + 10, task.id)

    # 绘制障碍
    if obstacle:
        circle = plt.Circle(obstacle.center, obstacle.radius, color="gray", alpha=0.3)
        plt.gca().add_patch(circle)
        plt.text(
            obstacle.center[0], obstacle.center[1], "Obstacle", fontsize=9, ha="center"
        )

    # 绘制无人机路径
    for i, drone in enumerate(drones):
        path = drone.path
        x, y = zip(*path)
        plt.plot(
            x,
            y,
            linestyle="-",
            marker="o",
            label=f"Drone {drone.id}",
            color=colors[i % len(colors)],
        )
        # plt.text(path[0][0], path[0][1], f"{drone.id} (Start)", fontsize=8)

    # 图形设置
    plt.title("无人机任务路径规划与协同")
    plt.xlabel("X 坐标 (m)")
    plt.ylabel("Y 坐标 (m)")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.tight_layout()
    plt.show()


def print_assignment(tasks, drones):
    print("任务分配方案：")
    for t in tasks:
        for d in drones:
            if t.coord in d.path:
                print(f"任务 {t.id} 分配给无人机 {d.id}")
                break

def solve_problem1():
    print("\n=== 问题1：最短总飞行距离覆盖所有目标点 ===")
    drones = [
        Drone("U1", 15, 500, 30),
        Drone("U2", 10, 600, 60),
        Drone("U3", 20, 450, 20),
    ]
    tasks = generate_tasks()
    obstacle = None
    assign_tasks(drones, tasks, obstacle)
    for drone in drones:
        print(
            f"Drone {drone.id} total time: {drone.time_used:.2f}s, path: {drone.path}"
        )
    total_distance = sum(
        sum(
            math.hypot(x2 - x1, y2 - y1)
            for (x1, y1), (x2, y2) in zip(drone.path[:-1], drone.path[1:])
        )
        for drone in drones
    )
    print(f"Total distance of all drones: {total_distance:.2f} meters")
    print_assignment(tasks, drones)
    visualize(drones, tasks, obstacle)

def solve_problem2():
    print("\n=== 问题2：动态障碍与紧急目标点 ===")
    drones = [
        Drone("U1", 15, 500, 30),
        Drone("U2", 10, 600, 60),
        Drone("U3", 20, 450, 20),
    ]
    tasks = generate_tasks()
    # 先执行100s
    obstacle = None
    assign_tasks(drones, tasks, obstacle)
    # 模拟100s后插入障碍和紧急任务
    print("\n[100s] 新增障碍和紧急目标点 T6")
    obstacle = Obstacle((900, 250), 100)
    t6 = Task("T6", (800, 600), "urgent", 1, 0)
    tasks.append(t6)
    # 标记未完成任务
    for t in tasks:
        t.assigned = False
    # 记录当前无人机状态
    for d in drones:
        d.current_pos = d.path[-1]
        d.path = [d.current_pos]
        d.time_used = 0
        d.remaining_time = d.max_time
    assign_tasks(drones, tasks, obstacle)
    for drone in drones:
        print(
            f"Drone {drone.id} total time: {drone.time_used:.2f}s, path: {drone.path}"
        )
    max_time = max(drone.time_used for drone in drones)
    print_assignment(tasks, drones)
    print(f"全覆盖最短时间: {max_time:.2f} 秒")
    visualize(drones, tasks, obstacle)

def generate_problem3_tasks():
    # 生成10个任务，约半数为非普通投放
    coords = [
        (700, 900), (1200, 800), (1000, 1000), (300, 300), (800, 600),
        (400, 800), (900, 400), (1300, 200), (200, 600), (600, 1200)
    ]
    types = ["urgent", "normal", "recon", "normal", "urgent", "recon", "normal", "urgent", "recon", "normal"]
    priorities = [1,2,3,2,1,3,2,1,3,2]
    demands = [8,12,40,6,10,30,5,7,50,9]
    tasks = []
    for i in range(10):
        ttype = types[i]
        demand = demands[i]
        if ttype == "recon":
            tasks.append(Task(f"T{i+1}", coords[i], ttype, priorities[i], demand))
        else:
            tasks.append(Task(f"T{i+1}", coords[i], ttype, priorities[i], demand))
    return tasks

def solve_problem3():
    print("\n=== 问题3：多类型任务、优先级与充电策略 ===")
    drones = [
        Drone("U1", 15, 500, 30),
        Drone("U2", 10, 600, 60),
        Drone("U3", 20, 450, 20),
    ]
    tasks = generate_problem3_tasks()
    obstacle = None
    assign_tasks(drones, tasks, obstacle)
    for drone in drones:
        print(
            f"Drone {drone.id} 总用时: {drone.time_used:.2f}s, 路径: {drone.path}, 充电次数: {drone.charge_cycle}"
        )
    print_assignment(tasks, drones)
    # 评估优先级满足度
    completed = {1:0, 2:0, 3:0}
    total = {1:0, 2:0, 3:0}
    for t in tasks:
        total[t.priority] += 1
        if t.assigned:
            completed[t.priority] += 1
    print("优先级满足度：")
    for k in sorted(total):
        print(f"优先级{k}: 完成{completed[k]}/{total[k]}")
    visualize(drones, tasks, obstacle)

# 主程序入口
if __name__ == "__main__":
    solve_problem1()
    solve_problem2()
    solve_problem3()
