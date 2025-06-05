import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as mplPolygon
from matplotlib.patches import Circle as mplCircle
from shapely.geometry import Polygon, Point

# 定义凸多边形与圆
polygon_coords = [(0, 0), (3, 0), (5, 5), (1,5), (0,3)]
polygon_coords_1 = [(0, 0), (5, 0), (5, 5), (0, 5)]
polygon = Polygon(polygon_coords)
circle_center = (7, 5)
circle_radius = 3
circle = Point(circle_center).buffer(circle_radius)

# 计算交集
intersection = polygon.intersection(circle)
area = intersection.area
print(area)

# 使用matplotlib绘制
fig, ax = plt.subplots()

# 绘制凸多边形
polygon_patch = mplPolygon(polygon_coords, edgecolor='blue', alpha=0.3, hatch="//")
ax.add_patch(polygon_patch)

# 绘制圆
circle_patch = mplCircle(circle_center, circle_radius, edgecolor='red', alpha=0.3, hatch="\\")
ax.add_patch(circle_patch)

# 绘制交集
if intersection.geom_type == "Polygon":
    coords = list(intersection.exterior.coords)
    intersection_patch = mplPolygon(coords, edgecolor='green', alpha=0.5)
    ax.add_patch(intersection_patch)
elif intersection.geom_type == "MultiPolygon":
    for geom in intersection:
        coords = list(geom.exterior.coords)
        intersection_patch = mplPolygon(coords, edgecolor='green', alpha=0.5)
        ax.add_patch(intersection_patch)

# 设置绘图区域与显示
ax.set_xlim(-1, 10)
ax.set_ylim(-1, 10)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Intersection of a convex polygon and a circle')
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
