import math
import heapq
from itertools import combinations
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from copy import deepcopy

# Define coordinates
points = {
    'depot': (0, 0),
    'T1': (1200, 800),
    'T2': (300, 450),
    'T3': (950, 200),
    'T4': (600, 1200),
    'T5': (1500, 500)
}

# UAV parameters
uav_speed = 50  # m/s
min_separation = 50  # m
max_communication = 1000  # m
max_flight_time = 600  # s

# Calculate Euclidean distance between two points
def calculate_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# Build distance matrix
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

# Clarke-Wright Savings algorithm implementation
def clarke_wright_savings(points, num_uavs):
    distance_matrix, locations = build_distance_matrix(points)
    depot_index = locations.index('depot')
    
    # Initialize routes
    routes = []
    for loc in locations:
        if loc != 'depot':
            routes.append([depot_index, locations.index(loc), depot_index])
    
    # Calculate savings
    savings = []
    for i, j in combinations([idx for idx in range(len(locations)) if idx != depot_index], 2):
        saving = distance_matrix[depot_index][i] + distance_matrix[depot_index][j] - distance_matrix[i][j]
        savings.append((saving, i, j))
    
    savings.sort(reverse=True, key=lambda x: x[0])
    
    # Merge routes
    for saving, i, j in savings:
        route_i = None
        route_j = None
        for route in routes:
            if i in route and j not in route and route.index(i) != 0 and route.index(i) != len(route)-1:
                route_i = route
            if j in route and i not in route and route.index(j) != 0 and route.index(j) != len(route)-1:
                route_j = route
        
        if route_i is not None and route_j is not None and len(routes) > num_uavs:
            new_route = []
            if route_i[-1] == depot_index and route_j[0] == depot_index:
                new_route = route_i[:-1] + route_j[1:]
            elif route_i[0] == depot_index and route_j[-1] == depot_index:
                new_route = route_j[:-1] + route_i[1:]
            
            if new_route:
                routes.remove(route_i)
                routes.remove(route_j)
                routes.append(new_route)
    
    # Convert to route details
    route_details = []
    total_distance = 0
    for route in routes:
        route_distance = 0
        for k in range(len(route)-1):
            route_distance += distance_matrix[route[k]][route[k+1]]
        total_distance += route_distance
        
        route_names = [locations[idx] for idx in route]
        route_details.append({
            'path': route_names,
            'distance': route_distance,
            'time': route_distance / uav_speed,
            'covered': [loc for loc in route_names if loc != 'depot']
        })
    
    return route_details, total_distance

# Check UAV separation constraints
def check_separation_constraints(routes, points, time_step=1):
    max_time = max(route['time'] for route in routes)
    time_points = np.arange(0, max_time + time_step, time_step)
    
    for t in time_points:
        positions = []
        for route in routes:
            pos = get_position_at_time(route, t)
            positions.append(pos)
        
        # Check all UAV pairs
        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                dist = calculate_distance(positions[i], positions[j])
                if dist < min_separation:
                    return False, f"UAV {i+1} and {j+1} at {t:.1f}s distance {dist:.1f}m < {min_separation}m"
                if dist > max_communication:
                    return False, f"UAV {i+1} and {j+1} at {t:.1f}s distance {dist:.1f}m > {max_communication}m"
    
    return True, "All separation constraints satisfied"

# Get UAV position at a specific time
def get_position_at_time(route, query_time):
    path = route['path']
    distance = route['distance']
    total_time = route['time']
    
    if query_time >= total_time:
        return points[path[-1]]
    
    elapsed_time = 0
    for i in range(len(path)-1):
        p1 = points[path[i]]
        p2 = points[path[i+1]]
        segment_dist = calculate_distance(p1, p2)
        segment_time = segment_dist / uav_speed
        
        if elapsed_time + segment_time >= query_time:
            ratio = (query_time - elapsed_time) / segment_time
            x = p1[0] + ratio * (p2[0] - p1[0])
            y = p1[1] + ratio * (p2[1] - p1[1])
            return (x, y)
        
        elapsed_time += segment_time
    
    return points[path[-1]]

# Adjust routes to satisfy constraints
def adjust_routes(routes, points):
    # Simple adjustment strategy: swap targets to make UAVs closer
    improved = True
    while improved:
        improved = False
        for i in range(len(routes)):
            for j in range(i+1, len(routes)):
                # Try swapping targets
                for k in range(1, len(routes[i]['path'])-1):
                    for l in range(1, len(routes[j]['path'])-1):
                        new_route_i = deepcopy(routes[i])
                        new_route_j = deepcopy(routes[j])
                        
                        # Swap targets
                        new_route_i['path'][k], new_route_j['path'][l] = new_route_j['path'][l], new_route_i['path'][k]
                        
                        # Recalculate distance and time
                        new_route_i['distance'] = calculate_path_distance(new_route_i['path'])
                        new_route_i['time'] = new_route_i['distance'] / uav_speed
                        new_route_j['distance'] = calculate_path_distance(new_route_j['path'])
                        new_route_j['time'] = new_route_j['distance'] / uav_speed
                        
                        # Check constraints
                        if (new_route_i['time'] <= max_flight_time and 
                            new_route_j['time'] <= max_flight_time):
                            new_routes = deepcopy(routes)
                            new_routes[i] = new_route_i
                            new_routes[j] = new_route_j
                            valid, _ = check_separation_constraints(new_routes, points)
                            if valid:
                                routes = new_routes
                                improved = True
                                break
                    if improved:
                        break
                if improved:
                    break
            if improved:
                break
    
    return routes

# Calculate total path distance
def calculate_path_distance(path):
    distance = 0
    for i in range(len(path)-1):
        p1 = points[path[i]]
        p2 = points[path[i+1]]
        distance += calculate_distance(p1, p2)
    return distance

# Visualize results
def plot_routes(routes, points):
    plt.figure(figsize=(12, 10))
    
    # Plot all points
    for name, coord in points.items():
        if name == 'depot':
            plt.plot(coord[0], coord[1], 'ro', markersize=10, label='Depot')
        else:
            plt.plot(coord[0], coord[1], 'bo', markersize=8)
            plt.text(coord[0]+20, coord[1]+20, name, fontsize=12)
    
    # Plot UAV routes
    colors = ['g', 'm', 'c', 'y', 'k']
    for i, route in enumerate(routes):
        path = route['path']
        x = [points[p][0] for p in path]
        y = [points[p][1] for p in path]
        plt.plot(x, y, colors[i], marker='o', linestyle='-', 
                 linewidth=2, label=f'UAV {i+1}: {route["distance"]:.1f}m')
    
    # Plot constraint range
    for i, route in enumerate(routes):
        for t in np.linspace(0, route['time'], 20):
            pos = get_position_at_time(route, t)
            circle = Circle(pos, min_separation/2, color=colors[i], alpha=0.1)
            plt.gca().add_patch(circle)
    
    plt.xlabel('X coordinate (m)')
    plt.ylabel('Y coordinate (m)')
    plt.title('UAV Route Planning (with Full Constraints)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Main program
if __name__ == "__main__":
    num_uavs = 3
    
    print("Initial route planning:")
    routes, total_distance = clarke_wright_savings(points, num_uavs)
    
    # Print initial results
    for i, route in enumerate(routes):
        print(f"\nUAV {i+1}:")
        print(f"Route: {' → '.join(route['path'])}")
        print(f"Distance: {route['distance']:.1f} m")
        print(f"Time: {route['time']:.1f} s")
        print(f"Covered targets: {', '.join(route['covered'])}")
    print(f"\nTotal flight distance: {total_distance:.1f} m")
    
    # Check constraints
    valid, message = check_separation_constraints(routes, points)
    print(f"\nConstraint check: {message}")
    
    # Adjust if needed
    if not valid:
        print("\nAdjusting routes to satisfy constraints...")
        adjusted_routes = adjust_routes(routes, points)
        valid, message = check_separation_constraints(adjusted_routes, points)
        print(f"Constraint check after adjustment: {message}")
        
        if valid:
            routes = adjusted_routes
            print("\nAdjusted route planning:")
            total_distance = sum(route['distance'] for route in routes)
            for i, route in enumerate(routes):
                print(f"\nUAV {i+1}:")
                print(f"Route: {' → '.join(route['path'])}")
                print(f"Distance: {route['distance']:.1f} m")
                print(f"Time: {route['time']:.1f} s")
                print(f"Covered targets: {', '.join(route['covered'])}")
            print(f"\nTotal flight distance: {total_distance:.1f} m")
        else:
            print("Warning: Not all constraints can be fully satisfied")
    
    # Visualization
    plot_routes(routes, points)