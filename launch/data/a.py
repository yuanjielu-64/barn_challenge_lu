import numpy as np
RADIUS = 0.075

def path_coord_to_gazebo_coord(x, y):
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


def load_and_print_npy(file_path):
    # 加载.npy文件
    data = np.load(file_path)

    # 打印文件内容
    print("Data from {}: \n".format(file_path), data)

    return data

if __name__ == "__main__":
    # 替换为你的.npy文件路径
    file_path = 'metrics_files/metrics_0.npy'
    path = load_and_print_npy(file_path)
    path_start = path[0]
    path_end = path[len(path)-1]

    start_x, start_y = path_coord_to_gazebo_coord(path_start[0], path_start[1])
    goal_x, goal_y = path_coord_to_gazebo_coord(path_end[0], path_end[1])

    # end point is currently provided in c-space, so we need to add in more distance
    # for it to be in the obstacle space
    # TODO - remove once start & end points are in obstacle space
    goal_y += 2 * RADIUS * 2
    start_y -= 1

    if start_x > -0.5:
        start_x = -0.5

    if start_x < -3.9:
        start_x = -3.9

    print("")