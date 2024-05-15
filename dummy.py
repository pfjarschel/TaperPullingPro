import matplotlib.pyplot as plt
import numpy as np

d0 = 125.0
l0 = 4000.0
x0 = 50000.0

alpha = np.linspace(1e-15, 1.0, 1001)
rw_a0 = (d0/2)*np.exp(-x0/(2*l0)) +0*alpha
rw_a = (d0/2)*((1 + alpha*x0/l0)**(-1/(2*alpha)))

plt.plot(alpha, rw_a0)
plt.plot(alpha, rw_a)
plt.show()