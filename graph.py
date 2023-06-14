import numpy as np
import matplotlib.pyplot as plt

# large input
# x = [int(i) for i in range(2, 22, 2)]
# y1 = [5.4e-06, 1.186e-05, 4.7719e-05, 0.000201389, 0.000750814, 0.00309949, 0.0171517, 0.0678672, 0.256858, 1.31979]
# y2 = [2.94e-06, 7.07e-06, 1.739e-05, 1.9359e-05, 3.107e-05, 0.000103769, 7.5179e-05, 8.56e-05, 0.000107499, 0.000154469]

# small input
x = [int(i) for i in range(2, 7)]
y1 = [4.619e-06, 9.69e-06, 1.245e-05, 2.464e-05, 3.968e-05]
y2 = [2.88e-06, 3.87e-06, 6.6e-06, 8.789e-06, 1.234e-05]

line1 = plt.plot(x, y1, label="exhaustive search")
line2 = plt.plot(x, y2, label="dynamic programming")

plt.ylabel('t')
plt.xlabel('n')
plt.legend()
plt.savefig("graph.png")
# plt.show()