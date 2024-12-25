import matplotlib.pyplot as plt

# 示例数据
ref_path_list_x = [0, 1, 2, 3, 4, 5]
ref_path_list_y = [0, 1, 4, 9, 16, 25]

# 绘制参考路径，虚线，蓝色，宽度为 1.0
plt.plot(ref_path_list_x, ref_path_list_y, '-.b', linewidth=1.0)

# 添加标题和标签
plt.title("Reference Path")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")

# 显示图形
plt.show()
