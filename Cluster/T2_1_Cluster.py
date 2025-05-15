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

    breakthrough = True
    while any(not t.assigned for t in unassigned):
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


# 主程序入口
if __name__ == "__main__":
    drones = [
        Drone("U1", 15, 500, 30),
        Drone("U2", 10, 600, 60),
        Drone("U3", 20, 450, 20),
    ]
    tasks = generate_tasks()
    # obstacle = Obstacle((900, 250), 100)  # 示例障碍
    obstacle = None  # 无障碍
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
    max_fly_time = max(drone.time_used for drone in drones)

    print(f"Total distance of all drones: {total_distance:.2f} meters")
    print(f"Maximum fly time among drones: {max_fly_time:.2f} seconds")
    visualize(drones, tasks, obstacle)
