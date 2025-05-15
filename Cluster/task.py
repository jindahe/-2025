# 任务类
class Task:
    def __init__(self, id, coord, type_, priority, demand):
        self.id = id
        self.coord = coord
        self.type = type_  # 'urgent', 'normal', 'recon'
        self.priority = priority
        self.demand = demand  # kg or hover seconds
        self.assigned = False
    def __repr__(self):
        return (
            f"Task(id={self.id}, coord={self.coord}, type={self.type}, "
            f"priority={self.priority}, demand={self.demand}, assigned={self.assigned})"
        )
