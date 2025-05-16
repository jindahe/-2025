import math
import heapq
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# 定义坐标点和障碍物
points = {
    'depot': (0, 0),
    'T1': (1200, 800),
    'T2': (300, 450),
    'T3': (950, 200),
    'T4': (600, 1200),
    'T5': (1500, 500),
    'T6': (800, 600)  # 新增目标点
}

obstacle = {
    'center': (900, 250),
    'radius': 100,
    'appear_time': 100  # 在100秒时出现
}

# 无人机速度 (m/s)
uav_speed = 50

# 计算两点之间的欧几里得距离
def calculate_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# 检查线段是否与圆形障碍物相交
def line_intersects_circle(p1, p2, circle_center, circle_radius):
    # 将线段表示为向量
    d = (p2[0]-p1[0], p2[1]-p1[1])
    f = (p1[0]-circle_center[0], p1[1]-circle_center[1])
    
    a = d[0]**2 + d[1]**2
    b = 2 * (f[0]*d[0] + f[1]*d[1])
    c = (f[0]**2 + f[1]**2) - circle_radius**2
    
    discriminant = b**2 - 4*a*c
    
    if discriminant < 0:
        return False  # 无交点
    
    discriminant = math.sqrt(discriminant)
    t1 = (-b - discriminant) / (2*a)
    t2 = (-b + discriminant) / (2*a)
    
    # 检查交点是否在线段上
    if 0 <= t1 <= 1 or 0 <= t2 <= 1:
        return True
    
    return False

# A*路径规划算法
def a_star_pathfinding(start, end, obstacle_center, obstacle_radius):
    # 定义网格大小
    grid_size = 50  # 米
    
    # 计算网格坐标
    def get_grid_coord(point):
        return (int(point[0]/grid_size), int(point[1]/grid_size))
    
    start_grid = get_grid_coord(start)
    end_grid = get_grid_coord(end)
    
    # 启发式函数 (曼哈顿距离)
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    
    # 邻居方向 (8个方向)
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    
    open_set = []
    heapq.heappush(open_set, (0, start_grid))
    
    came_from = {}
    g_score = {start_grid: 0}
    f_score = {start_grid: heuristic(start_grid, end_grid)}
    
    open_set_hash = {start_grid}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        open_set_hash.remove(current)
        
        if current == end_grid:
            # 重建路径
            path = []
            while current in came_from:
                path.append((current[0]*grid_size, current[1]*grid_size))
                current = came_from[current]
            path.append(start)
            path.reverse()
            path.append(end)
            return path
        
        for dx, dy in neighbors:
            neighbor = (current[0]+dx, current[1]+dy)
            
            # 转换为实际坐标
            neighbor_pos = (neighbor[0]*grid_size, neighbor[1]*grid_size)
            
            # 检查是否在障碍物内
            if calculate_distance(neighbor_pos, obstacle_center) <= obstacle_radius:
                continue
            
            # 计算临时g分数
            temp_g_score = g_score[current] + calculate_distance(
                (current[0]*grid_size, current[1]*grid_size),
                neighbor_pos
            )
            
            if neighbor not in g_score or temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + heuristic(neighbor, end_grid)
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    return None  # 没有找到路径

# 调整路径以避开障碍物
def adjust_path_for_obstacle(original_path, obstacle_center, obstacle_radius):
    adjusted_path = [original_path[0]]
    
    for i in range(len(original_path)-1):
        p1 = points[original_path[i]]
        p2 = points[original_path[i+1]]
        
        if line_intersects_circle(p1, p2, obstacle_center, obstacle_radius):
            # 需要避障
            new_segment = a_star_pathfinding(p1, p2, obstacle_center, obstacle_radius)
            if new_segment:
                # 将新路径段转换为目标点名称
                for point in new_segment[1:-1]:  # 跳过起点和终点(已经在路径中)
                    adjusted_path.append(f"WP{point[0]}_{point[1]}")  # 添加航路点
            else:
                print(f"Warning: Could not find path between {original_path[i]} and {original_path[i+1]}")
        adjusted_path.append(original_path[i+1])
    
    return adjusted_path

# 计算路径总距离
def calculate_path_distance(path):
    distance = 0
    for i in range(len(path)-1):
        p1 = path[i] if isinstance(path[i], tuple) else points[path[i]]
        p2 = path[i+1] if isinstance(path[i+1], tuple) else points[path[i+1]]
        distance += calculate_distance(p1, p2)
    return distance

# 计算到达各点的时间
def calculate_arrival_times(path, start_time=0):
    current_time = start_time
    arrival_times = {}
    positions = []
    
    for i in range(len(path)-1):
        p1 = path[i] if isinstance(path[i], tuple) else points[path[i]]
        p2 = path[i+1] if isinstance(path[i+1], tuple) else points[path[i+1]]
        
        distance = calculate_distance(p1, p2)
        travel_time = distance / uav_speed
        
        if isinstance(path[i+1], str) and path[i+1].startswith('T'):
            arrival_times[path[i+1]] = current_time + travel_time
        
        positions.append((p1, current_time))
        current_time += travel_time
    
    positions.append((points[path[-1]], current_time))
    return arrival_times, positions, current_time

# 主程序
if __name__ == "__main__":
    # 初始路径分配 (基于问题1的解决方案)
    original_routes = [
        ['depot', 'T1', 'T5', 'depot'],
        ['depot', 'T2', 'T3', 'depot'],
        ['depot', 'T4', 'depot']
    ]
    
    # 调整路径以避开障碍物
    adjusted_routes = []
    for route in original_routes:
        adjusted_route = adjust_path_for_obstacle(route, obstacle['center'], obstacle['radius'])
        adjusted_routes.append(adjusted_route)
    
    # 分配新增目标点T6给飞行时间最短的无人机
    # 计算各无人机完成初始任务的时间
    completion_times = []
    for route in adjusted_routes:
        _, _, time = calculate_arrival_times(route)
        completion_times.append(time)
    
    # 选择最早完成的无人机来服务T6
    selected_uav = completion_times.index(min(completion_times))
    
    # 将T6插入到选中的无人机路径中
    # 找到最佳插入位置 (使总距离增加最少)
    best_position = 1  # 默认插入在第一个目标点之后
    min_increase = float('inf')
    target_route = adjusted_routes[selected_uav]
    
    for i in range(1, len(target_route)):
        # 计算在位置i插入T6的距离增加量
        original_dist = calculate_distance(
            points[target_route[i-1]], 
            points[target_route[i]]
        )
        new_dist = (
            calculate_distance(points[target_route[i-1]], points['T6']) +
            calculate_distance(points['T6'], points[target_route[i]])
        )
        increase = new_dist - original_dist
        
        if increase < min_increase:
            min_increase = increase
            best_position = i
    
    # 插入T6
    adjusted_routes[selected_uav].insert(best_position, 'T6')
    
    # 重新计算调整后的路径
    final_routes = []
    for route in adjusted_routes:
        # 确保路径以depot开始和结束
        if route[0] != 'depot':
            route.insert(0, 'depot')
        if route[-1] != 'depot':
            route.append('depot')
        
        # 展开航路点
        expanded_route = []
        for point in route:
            if point.startswith('WP'):
                coords = point[2:].split('_')
                expanded_route.append((float(coords[0]), float(coords[1])))
            else:
                expanded_route.append(point)
        final_routes.append(expanded_route)
    
    # 计算各无人机的时间分配
    print("Adjusted UAV Paths with Obstacle Avoidance:")
    max_completion_time = 0
    for i, route in enumerate(final_routes):
        arrival_times, positions, total_time = calculate_arrival_times(route)
        
        print(f"\nUAV {i+1}:")
        print(f"Path: {' → '.join([p if isinstance(p, str) else f'WP({p[0]},{p[1]})' for p in route])}")
        print(f"Total distance: {calculate_path_distance(route):.1f} meters")
        print(f"Total time: {total_time:.1f} seconds")
        
        # 打印目标点到达时间
        for target, time in arrival_times.items():
            print(f"  Arrives at {target} at {time:.1f}s")
        
        max_completion_time = max(max_completion_time, total_time)
    
    print(f"\nTime to complete all targets: {max_completion_time:.1f} seconds")
    
    # 可视化结果
    plt.figure(figsize=(12, 10))
    
    # 绘制所有点
    for name, coord in points.items():
        if name == 'depot':
            plt.plot(coord[0], coord[1], 'ro', markersize=10, label='Depot')
        else:
            plt.plot(coord[0], coord[1], 'bo', markersize=8, label=name)
            plt.text(coord[0]+20, coord[1]+20, name, fontsize=12)
    
    # 绘制障碍物
    obstacle_circle = Circle(
        obstacle['center'], obstacle['radius'],
        color='r', alpha=0.3, label='Obstacle (appears at 100s)'
    )
    plt.gca().add_patch(obstacle_circle)
    
    # 绘制路线
    colors = ['g', 'm', 'c', 'y', 'k']
    for i, route in enumerate(final_routes):
        x_coords = []
        y_coords = []
        for point in route:
            if isinstance(point, tuple):
                x_coords.append(point[0])
                y_coords.append(point[1])
            else:
                x_coords.append(points[point][0])
                y_coords.append(points[point][1])
        
        plt.plot(x_coords, y_coords, colors[i % len(colors)], 
                 linestyle='-', linewidth=2, 
                 label=f'UAV {i+1} Path')
    
    plt.xlabel('X Coordinate (m)')
    plt.ylabel('Y Coordinate (m)')
    plt.title('UAV Path Planning with Obstacle Avoidance')
    plt.legend()
    plt.grid(True)
    plt.show()