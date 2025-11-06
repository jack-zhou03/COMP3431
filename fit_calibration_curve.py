import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# 替换为你记录的实验数据
h = np.array([180, 120, 90, 75, 62])
D = np.array([0.4, 0.6, 0.8, 1.0, 1.2])  # 单位：m

def model(h, a, b):
    return a * np.power(h, -b)

params, _ = curve_fit(model, h, D)
a, b = params
print(f"拟合结果: distance = {a:.3f} * h^(-{b:.3f})")

# 可视化
plt.scatter(h, D, label="data")
plt.plot(h, model(h, a, b), color="orange", label=f"fit: {a:.2f}*h^-{b:.3f}")
plt.xlabel("Height (pixels)")
plt.ylabel("Distance (m)")
plt.legend()
plt.show()
