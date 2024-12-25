import numpy as np
import matplotlib.pyplot as plt


ref_path_data = np.loadtxt('data.txt')

x = ref_path_data[:, 0]
y = ref_path_data[:, 1]

plt.plot(x, y, marker='o', linestyle='-', color='b', label='Data points')

plt.title('Plot')
plt.xlabel('X')
plt.ylabel('Y')

plt.legend()

plt.show()
