import cv2
import numpy as np
from shapely.geometry import Polygon, MultiPolygon
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
from datetime import datetime


def pgm_to_obstacle_polygons(pgm_path):
    # 读取PGM图像
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise FileNotFoundError(f"无法读取图像: {pgm_path}")

    # 找到所有的黑色区域（障碍物）
    _, binary_img = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY_INV)

    # 查找轮廓
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 将轮廓转换为Shapely多边形
    polygons = []
    for contour in contours:
        # 轮廓中的每个点是一个(x, y)坐标
        points = [tuple(point[0]) for point in contour]
        # 创建多边形并添加到列表中
        polygon = Polygon(points)
        polygons.append(polygon)

    # 创建一个多多边形对象
    obstacle_map = MultiPolygon(polygons)

    return obstacle_map


def plot_obstacle_map(obstacle_map):
    fig, ax = plt.subplots()

    # 创建Matplotlib的多边形对象
    patches = []
    for polygon in obstacle_map.geoms:  # 使用 obstacle_map.geoms 迭代多边形
        mpl_poly = MplPolygon(np.array(polygon.exterior.coords), closed=True)
        patches.append(mpl_poly)

    # 添加多边形到图像中
    p = PatchCollection(patches, facecolor='black', edgecolor='black', alpha=0.5)
    ax.add_collection(p)

    # 设置图像的显示范围
    ax.set_xlim([0, max(max(polygon.exterior.coords.xy[0]) for polygon in obstacle_map.geoms)])
    ax.set_ylim([0, max(max(polygon.exterior.coords.xy[1]) for polygon in obstacle_map.geoms)])
    ax.set_aspect('equal')
    plt.gca().invert_yaxis()  # 使图像的原点在左上角，与图像坐标系一致

    plt.title('Obstacle Map')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()

# 使用示例
pgm_path = 'map_files/map_pgm_0.pgm'
obstacle_polygons = pgm_to_obstacle_polygons(pgm_path)
plot_obstacle_map(obstacle_polygons)
# 打印障碍物多边形
print(obstacle_polygons)
