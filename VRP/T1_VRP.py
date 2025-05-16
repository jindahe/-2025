#!/bin/env python3
import math
from itertools import combinations
import matplotlib.pyplot as plt

# 定义坐标点
points = {
    'depot': (0, 0),
    'T1': (1200, 800),
    'T2': (300, 450),
    'T3': (950, 200),
    'T4': (600, 1200),
    'T5': (1500, 500)
}

# 计算两点之间的欧几里得距离
def calculate_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# 构建距离矩阵
def build_distance_matrix(points):
    locations = list(points.keys())
    n = len(locations)
    distance_matrix = [[0]*n for _ in range(n)]
    
    for i in range(n):
        for j in range(n):
            if i != j:
                distance_matrix[i][j] = calculate_distance(
                    points[locations[i]], 
                    points[locations[j]]
                )
    return distance_matrix, locations

# 节约算法实现
def clarke_wright_savings(points, num_vehicles):
    # 构建距离矩阵和位置列表
    distance_matrix, locations = build_distance_matrix(points)
    depot_index = locations.index('depot')
    
    # 初始化路线：每个目标点作为一个单独的路线
    routes = []
    for loc in locations:
        if loc != 'depot':
            routes.append([depot_index, locations.index(loc), depot_index])
    
    # 计算所有节约值
    savings = []
    for i, j in combinations([idx for idx in range(len(locations)) if idx != depot_index], 2):
        saving = distance_matrix[depot_index][i] + distance_matrix[depot_index][j] - distance_matrix[i][j]
        savings.append((saving, i, j))
    
    # 按节约值从大到小排序
    savings.sort(reverse=True, key=lambda x: x[0])
    
    # 合并路线
    for saving, i, j in savings:
        # 找到包含i和j的路线
        route_i = None
        route_j = None
        for route in routes:
            if i in route and j not in route and route.index(i) != 0 and route.index(i) != len(route)-1:
                route_i = route
            if j in route and i not in route and route.index(j) != 0 and route.index(j) != len(route)-1:
                route_j = route
        
        # 如果两条路线可以合并
        if route_i is not None and route_j is not None and len(routes) > num_vehicles:
            # 合并路线
            new_route = []
            if route_i[-1] == depot_index and route_j[0] == depot_index:
                new_route = route_i[:-1] + route_j[1:]
            elif route_i[0] == depot_index and route_j[-1] == depot_index:
                new_route = route_j[:-1] + route_i[1:]
            
            if new_route:
                routes.remove(route_i)
                routes.remove(route_j)
                routes.append(new_route)
    
    # 计算每条路线的总距离
    route_details = []
    total_distance = 0
    for route in routes:
        route_distance = 0
        for i in range(len(route)-1):
            route_distance += distance_matrix[route[i]][route[i+1]]
        total_distance += route_distance
        
        # 转换为目标点名称
        route_names = [locations[idx] for idx in route]
        route_details.append({
            'path': route_names,
            'distance': route_distance,
            'covered': [loc for loc in route_names if loc != 'depot']
        })
    
    return route_details, total_distance

# 可视化结果
def plot_routes(points, routes):
    plt.figure(figsize=(10, 8))
    
    # 绘制所有点
    for name, coord in points.items():
        if name == 'depot':
            plt.plot(coord[0], coord[1], 'ro', markersize=10, label='Depot')
        else:
            plt.plot(coord[0], coord[1], 'bo', markersize=8, label=name)
            plt.text(coord[0]+20, coord[1]+20, name, fontsize=12)
    
    # 绘制路线
    colors = ['g', 'm', 'c', 'y', 'k']
    for i, route in enumerate(routes):
        path = route['path']
        x_coords = [points[loc][0] for loc in path]
        y_coords = [points[loc][1] for loc in path]
        plt.plot(x_coords, y_coords, colors[i % len(colors)], 
                 linestyle='-', linewidth=2, 
                 label=f'UAV {i+1}: {route["distance"]:.1f}m')
    
    plt.xlabel('X Coordinate (m)')
    plt.ylabel('Y Coordinate (m)')
    plt.title('UAV Path Planning with VRP Algorithm')
    plt.legend()
    plt.grid(True)
    plt.show()

# 主程序
if __name__ == "__main__":
    num_uavs = 3
    
    # 使用节约算法求解
    routes, total_distance = clarke_wright_savings(points, num_uavs)
    
    # 打印结果
    print("Optimal UAV Path Assignment:")
    for i, route in enumerate(routes):
        print(f"UAV {i+1}:")
        print(f"  Path: {' → '.join(route['path'])}")
        print(f"  Distance: {route['distance']:.1f} meters")
        print(f"  Covered targets: {', '.join(route['covered'])}")
        print()
    
    print(f"Total flight distance: {total_distance:.1f} meters")
    
    # 验证所有目标点是否被覆盖
    all_covered = set()
    for route in routes:
        all_covered.update(route['covered'])
    
    if len(all_covered) == len(points)-1:  # 减去depot
        print("All targets are covered!")
    else:
        missing = set(points.keys()) - {'depot'} - all_covered
        print(f"Warning: Missing targets - {', '.join(missing)}")
    
    # 可视化结果
    plot_routes(points, routes)
