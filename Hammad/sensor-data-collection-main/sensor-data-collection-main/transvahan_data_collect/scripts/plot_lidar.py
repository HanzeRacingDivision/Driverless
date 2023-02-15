import numpy as np
import matplotlib.pyplot as plt

scan = 0

data = np.load("/media/amm/Backup/acads/ARTPARK/test/transvahan/Data/2021-12-09/run1_lidar/%05d.npy"%(scan))

lidar=[]
alpha=(0)
rot = np.array([[np.cos(alpha),np.sin(alpha)],
                [-np.sin(alpha),np.cos(alpha)]])
for i in range(data.shape[0]):
    temp=np.array([data[i,0],data[i,1]])
    lidar.append(np.dot(temp,rot))

lidar=np.array(lidar)
plt.scatter(lidar[:,0],lidar[:,1],s=0.1)
plt.show()