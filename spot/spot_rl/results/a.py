import numpy as np
import matplotlib.pyplot as plt

data = np.load("spot_ars_rand_seed0.npy")

plt.figure(figsize=(10,6))
plt.plot(data[:,0], label="Clearance")
plt.plot(data[:,1], label="Body height")
plt.legend()
plt.show()
