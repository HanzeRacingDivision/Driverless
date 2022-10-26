import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from sklearn.feature_extraction import DictVectorizer

def vectorizing(coordinates): 
    
    dictvectorizer = DictVectorizer(dtype = np.float64 ,sparse= False)
    features = dictvectorizer.fit_transform(coordinates)
    final_data = np.delete(features,(0,1,2,5),axis = 1)
    return final_data

output = vectorizing() # here you should put a set of coordinates
triangles = Delaunay(output)
plt.triplot(output[:,0], output[:,1], triangles.simplices)
plt.plot(output[:,0], output[:,1], 'o')
plt.show()
