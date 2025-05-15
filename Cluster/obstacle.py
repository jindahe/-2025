# 障碍物
class Obstacle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def is_blocked(self, p1, p2):
        # 判断线段 p1-p2 是否穿过障碍圆（近似）
        """
        Determine if the line segment p1-p2 intersects with the circle defined by center and radius.

        Parameters:
        - p1: tuple (x1, y1) for the segment start point.
        - p2: tuple (x2, y2) for the segment end point.
        - center: tuple (cx, cy) for the circle center.
        - radius: circle radius.

        Returns:
        - True if the segment intersects the circle, False otherwise.
        """
        x1, y1 = p1
        x2, y2 = p2
        cx, cy = self.center

        # Vector from p1 to p2
        dx, dy = x2 - x1, y2 - y1
        # Vector from p1 to circle center
        fx, fy = x1 - cx, y1 - cy

        # Compute projection parameter t of center onto the line
        t = -(fx * dx + fy * dy) / (dx * dx + dy * dy)
        # Clamp t to the segment [0, 1]
        t = max(0, min(1, t))

        # Find the closest point on the segment to the circle center
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Distance from closest point to circle center
        dist_sq = (closest_x - cx) ** 2 + (closest_y - cy) ** 2

        # Intersection if distance <= radius^2
        return dist_sq <= self.radius**2
