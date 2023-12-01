import numpy as np
import matplotlib.pyplot as plt

# read txt file
data = np.loadtxt('../report/point.txt')

# extraction of data in horizontal and vertical coordinates
x = data[[0, 2, 4, 6, 8, 10, 12, 14, 16, 18],:]
y = data[[1, 3, 5, 7, 9, 11, 13, 15, 17, 19],:]

plt.axis('equal')

for i in range(0, x.shape[1], 8500):
    plt.plot(np.append(x[:4, i], x[0, i]), np.append(y[:4, i], y[0, i]), 'k-', linewidth=1/3)  # connect points 1, 2, 3, 4, 1
    plt.plot(np.append(x[6:10, i], x[6, i]), np.append(y[6:10, i], y[6, i]), 'k-', linewidth=1/3)  # connect points 7, 8, 9, 10, 7
    plt.plot([x[4, i], x[5, i]], [y[4, i], y[5, i]], 'k-', linewidth=1/3)  # connect points 5 and 6

plt.savefig('../report/trajectory.png', dpi=2000)
plt.show()
