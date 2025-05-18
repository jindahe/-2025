import math
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt

class Drone:
    def __init__(self, name, max_load, max_endurance, hover_time):
        self.name = name
        self.max_load = max_load
        self.max_endurance = max_endurance
        self.hover_time = hover_time
        self.current_load = 0
        self.remaining_endurance = max_endurance
        self.mission_log = []
        self.position = (0, 0)  # 起始位置为基地(0,0)
        self.total_distance = 0

    def can_assign(self, task):
        """检查是否能分配任务"""
        if task.task_type in ['紧急投放', '普通投放']:
            return (self.current_load + task.requirement <= self.max_load and 
                    self.remaining_endurance > 0)
        else:  # 侦察任务
            return self.remaining_endurance > task.requirement

    def assign_task(self, task):
        """分配任务给无人机"""
        distance = self.calculate_distance(task.location)
        flight_time = distance / 50  # 假设飞行速度50m/s
        
        if task.task_type in ['紧急投放', '普通投放']:
            self.current_load += task.requirement
            time_required = flight_time
        else:  # 侦察任务
            time_required = flight_time + task.requirement
        
        # 检查是否需要充电
        if self.remaining_endurance < time_required + self.calculate_distance((0,0)) / 50:
            self.return_to_base()
        
        self.remaining_endurance -= time_required
        self.total_distance += distance
        self.position = task.location
        self.mission_log.append({
            'task': task,
            'start_time': self.current_time(),
            'end_time': self.current_time() + time_required,
            'distance': distance
        })
    
    def return_to_base(self):
        """返回基地充电"""
        distance = self.calculate_distance((0,0))
        time_required = distance / 50 + 60  # 飞行时间+充电时间
        
        self.remaining_endurance = self.max_endurance
        self.current_load = 0
        self.total_distance += distance
        self.position = (0, 0)
        self.mission_log.append({
            'task': None,
            'description': 'Return to base and charge',
            'start_time': self.current_time(),
            'end_time': self.current_time() + time_required,
            'distance': distance
        })
    
    def calculate_distance(self, target):
        """计算当前位置到目标的距离"""
        return math.sqrt((self.position[0]-target[0])**2 + (self.position[1]-target[1])**2)
    
    def current_time(self):
        """计算当前累计任务时间"""
        if not self.mission_log:
            return 0
        return self.mission_log[-1]['end_time']

class Task:
    def __init__(self, task_id, task_type, location, priority, requirement):
        self.task_id = task_id
        self.task_type = task_type
        self.location = location
        self.priority = priority
        self.requirement = requirement
        self.assigned = False

def generate_tasks():
    """生成模拟任务"""
    tasks = [
        Task(1, '紧急投放', (500, 800), 1, 5),
        Task(2, '普通投放', (1500, 200), 2, 10),
        Task(3, '侦察任务', (200, 1500), 3, 20),
        Task(4, '紧急投放', (1200, 1000), 1, 8),
        Task(5, '普通投放', (300, 600), 2, 7),
        Task(6, '侦察任务', (1800, 700), 3, 30),
        Task(7, '紧急投放', (100, 1000), 1, 6),
        Task(8, '普通投放', (1600, 400), 2, 9),
        Task(9, '侦察任务', (800, 300), 3, 25),
        Task(10, '普通投放', (1400, 900), 2, 11)
    ]
    return tasks

def assign_tasks(drones, tasks):
    """任务分配主算法，严格按照T3.md的分层贪心思想"""
    # 按优先级分组
    priority_groups = defaultdict(list)
    for task in tasks:
        priority_groups[task.priority].append(task)
    # 优先级1：每个无人机分配一个紧急任务
    urgent_tasks = priority_groups.get(1, [])
    for i, task in enumerate(urgent_tasks):
        if i < len(drones):
            drones[i].assign_task(task)
            task.assigned = True
    # 优先级2：普通投放，分三类，遍历所有分配方案，选最短完成时间
    normal_tasks = priority_groups.get(2, [])
    from itertools import permutations
    best_perm = None
    best_time = float('inf')
    if len(normal_tasks) == len(drones):
        for perm in permutations(normal_tasks):
            # 复制无人机状态
            drones_copy = [Drone(d.name, d.max_load, d.max_endurance, d.hover_time) for d in drones]
            for i, task in enumerate(perm):
                drones_copy[i].position = drones[i].position
                drones_copy[i].remaining_endurance = drones[i].remaining_endurance
                drones_copy[i].current_load = drones[i].current_load
                drones_copy[i].mission_log = list(drones[i].mission_log)
                drones_copy[i].total_distance = drones[i].total_distance
                drones_copy[i].assign_task(task)
            finish_time = max(d.current_time() for d in drones_copy)
            if finish_time < best_time:
                best_time = finish_time
                best_perm = perm
        # 按最佳分配方案分配
        for i, task in enumerate(best_perm):
            drones[i].assign_task(task)
            task.assigned = True
    else:
        # 普通贪心分配
        for task in normal_tasks:
            best_drone = min(drones, key=lambda d: d.current_time())
            best_drone.assign_task(task)
            task.assigned = True
    # 优先级3：侦察任务，遍历所有无人机，分配给完成普通任务后最早空闲的无人机
    scout_tasks = priority_groups.get(3, [])
    for task in scout_tasks:
        best_drone = None
        best_time = float('inf')
        for drone in drones:
            # 计算该无人机完成普通任务后到侦察点的时间
            temp_drone = Drone(drone.name, drone.max_load, drone.max_endurance, drone.hover_time)
            temp_drone.position = drone.position
            temp_drone.remaining_endurance = drone.remaining_endurance
            temp_drone.current_load = drone.current_load
            temp_drone.mission_log = list(drone.mission_log)
            temp_drone.total_distance = drone.total_distance
            temp_drone.assign_task(task)
            finish_time = temp_drone.current_time()
            if finish_time < best_time:
                best_time = finish_time
                best_drone = drone
        if best_drone:
            best_drone.assign_task(task)
            task.assigned = True
    # 处理未分配的任务（可能由于约束无法分配）
    unassigned = [t for t in tasks if not t.assigned]
    if unassigned:
        print(f"警告：{len(unassigned)}个任务未能分配")

def visualize_schedule(drones):
    """可视化任务调度结果，风格与T2/T1统一"""
    plt.figure(figsize=(10, 8))
    # 绘制所有任务点
    all_points = [(0, 0)]
    for drone in drones:
        for mission in drone.mission_log:
            if mission['task']:
                all_points.append(mission['task'].location)
    all_points = list(set(all_points))
    for idx, (x, y) in enumerate(all_points):
        if (x, y) == (0, 0):
            plt.plot(x, y, 'ro', markersize=10, label='Depot')
        else:
            if idx == 1:
                plt.plot(x, y, 'bo', markersize=8, label='Target')
            else:
                plt.plot(x, y, 'bo', markersize=8)
            plt.text(x+20, y+20, f'({x},{y})', fontsize=12)
    # 绘制无人机路径
    colors = ['g', 'm', 'c', 'y', 'k']
    for i, drone in enumerate(drones):
        x = [0]
        y = [0]
        for mission in drone.mission_log:
            if mission['task']:
                x.append(mission['task'].location[0])
                y.append(mission['task'].location[1])
        plt.plot(x, y, colors[i % len(colors)], linestyle='-', linewidth=2, label=f'UAV {i+1}: {sum([math.sqrt((x[j]-x[j-1])**2 + (y[j]-y[j-1])**2) for j in range(1, len(x))]):.1f}m')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('UAV Path Planning ')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def print_results(drones, tasks):
    """打印结果统计"""
    print("\n=== 任务分配结果 ===")
    for drone in drones:
        print(f"\n{drone.name}:")
        print(f"总飞行距离: {drone.total_distance:.1f}m")
        print(f"剩余续航时间: {drone.remaining_endurance:.1f}s")
        print("任务序列:")
        for mission in drone.mission_log:
            if mission['task']:
                print(f"  任务{mission['task'].task_id}: {mission['task'].task_type} "
                      f"在({mission['task'].location[0]},{mission['task'].location[1]}), "
                      f"时间{mission['start_time']:.1f}-{mission['end_time']:.1f}s")
            else:
                print(f"  返回基地充电: {mission['start_time']:.1f}-{mission['end_time']:.1f}s")
    
    completion_times = [d.current_time() for d in drones]
    print(f"\n总任务完成时间: {max(completion_times):.1f}s")
    
    unassigned = [t for t in tasks if not t.assigned]
    if unassigned:
        print(f"\n未分配任务: {[t.task_id for t in unassigned]}")

def main():
    # 初始化无人机
    drones = [
        Drone('U1', 15, 500, 30),
        Drone('U2', 10, 600, 60),
        Drone('U3', 20, 450, 20)
    ]
    
    # 生成任务
    tasks = generate_tasks()
    
    # 分配任务
    assign_tasks(drones, tasks)
    
    # 输出结果
    print_results(drones, tasks)
    
    # 可视化
    visualize_schedule(drones)

if __name__ == "__main__":
    main()