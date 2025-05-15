from common import *
import math


# 无人机类
class Drone:
    def __init__(self, id, weight_limit, endurance, hover_time):
        self.id = id
        self.weight_limit = weight_limit
        self.max_time = endurance
        self.hover_time = hover_time
        self.path = [BASE]
        self.time_used = 0
        self.load_used = 0
        self.current_pos = BASE
        self.is_leader = False
        self.remaining_time = self.max_time
        self.charge_cycle = 0  # 充电次数

    def distance(self, pos):
        return math.hypot(self.current_pos[0] - pos[0], self.current_pos[1] - pos[1])

    def move_to(self, pos, task_time=0):
        dist = self.distance(pos)
        flight_time = dist / 50
        total_time = flight_time + task_time

        if self.remaining_time < total_time + self.distance(BASE) / 50:
            return False  # 无法执行该任务

        self.remaining_time -= total_time
        self.time_used += total_time
        self.current_pos = pos
        self.path.append(pos)
        return True

    def return_to_base(self):
        rt_time = self.distance(BASE) / 50
        if self.remaining_time < rt_time:
            print(f"[警告] {self.id} 电量不足，无法返回")
            return False
        self.time_used += rt_time
        self.remaining_time -= rt_time
        self.current_pos = BASE
        self.path.append(BASE)
        self.charge_cycle += 1
        self.remaining_time = self.max_time  # 充电
        self.time_used += 60  # 充电时间
        return True

    def __repr__(self):
        return (
            f"Drone(id={self.id}, weight_limit={self.weight_limit}, "
            f"endurance={self.max_time}, hover_time={self.hover_time}, "
            f"path={self.path}, time_used={self.time_used:.2f}, "
            f"load_used={self.load_used}, current_pos={self.current_pos}, "
            f"is_leader={self.is_leader})"
        )
def check_need_return(drones: list[Drone]):
    time_to_base = [d.distance(BASE) / 50 for d in drones]
    min_margin = [d.remaining_time - t for d, t in zip(drones, time_to_base)]
    if any(m < 20 for m in min_margin):  # 保险余量，比如 20s
        return True
    return False

# 统一返航，延时降落以拉开间距
def all_return_to_base(drones: list[Drone]):
    for i, d in enumerate(drones):
        d.return_to_base()
        d.time_used += i * 2  # 延时降落间隔 ≥2s，模拟拉开50m
