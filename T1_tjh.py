import math
import itertools

points = [
    (1200, 800),
    (300, 450),
    (950, 200),
    (600, 1200),
    (1500, 500),  
]

def Euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def distance_group(group_indices):
    if not group_indices:
        return 0.0
    group_points = [points[i] for i in group_indices]
    min_dist = float('inf')
    for perm in itertools.permutations(group_points):
        current_dist = Euclidean_distance((0, 0), perm[0])
        for i in range(1, len(perm)):
            current_dist += Euclidean_distance(perm[i-1], perm[i])
        if current_dist < min_dist:
            min_dist = current_dist
    return min_dist

groups = []

for three in itertools.combinations(range(5), 3):
    remain_points = list(set(range(5)) - set(three))
    groups.append([list(three), [remain_points[0]], [remain_points[1]]])

for single in range(5):  
    remaining = list(set(range(5)) - {single})
    pairs = []
    seen = set()
    for pair in itertools.combinations(remaining, 2):  
        sorted_pair = tuple(sorted(pair))
        complement = tuple(sorted(set(remaining) - set(pair)))
        if (sorted_pair, complement) not in seen and (complement, sorted_pair) not in seen:
            seen.add((sorted_pair, complement))
            pairs.append((list(sorted_pair), list(complement)))
    for pair1, pair2 in pairs[:3]:
        groups.append([[single], pair1, pair2])

min_total = float('inf')
best_group = None
for group in groups:
    total = 0.0
    for subgroup in group:
        total += distance_group(subgroup)
    if total < min_total:
        min_total = total
        best_group = group

def indices_change(indices):
    return [f'T{i+1}' for i in indices]

best_group_name = []
for subgroup in best_group:
    subgroup_names = indices_change(subgroup)
    best_group_name.append(subgroup_names)

print(f"无人机最小总飞行距离为 {min_total:.2f} 米")
print("对应分组方案为：")
for i, subgroup in enumerate(best_group_name, 1):
    print(f"无人机{i}目标点：{','.join(subgroup)}")