import numpy as np
from matplotlib import pyplot as plt

x = np.linspace(0, 1, 100)
powers = np.linspace(0.5, 2.0, 13).tolist()
for power in powers:
    plt.plot(500 * x, 500 * np.power(x, power), label=f'{power}')
    
plt.grid()
plt.legend()
plt.show()