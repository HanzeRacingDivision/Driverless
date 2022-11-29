import numpy as np
from control.path_finding import path_finding

x = np.array([[[0, 1], [1, 1]], [[1, 1], [0, 2]]])
print(path_finding(x))
