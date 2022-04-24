import numpy as np
import matplotlib.pyplot as plt


# Random Sample Consensus
def ransac(data, inliers, model, threshold):


    return inliers, model


# plane normal from three points
def surfNorm(xyz):

    U = xyz[2,:] - xyz[1,:]
    V = xyz[3,:] - xyz[1,:]
    normal = np.cross(U,V) / np.norm(np.cross(U,V))
    
    return normal


# Using readlines()
file1 = open('Final_Project/points.ply', 'r')
Lines = file1.readlines()

length = 0
# Strips the newline character
for line in Lines:
    length += 1

num = int(np.floor( (length-11)/2 ))
print(num)
x = np.zeros((num,1), dtype=np.float64)
y = np.zeros((num,1), dtype=np.float64)
z = np.zeros((num,1), dtype=np.float64)

count = 0
i = 0
for line in Lines:
    count += 1
    if count > 11:
        if count % 2 == 0:
            point = line.split(' ')
            #print(point[0], point[1], point[2])
            #print(i)
            x[i] = point[0]
            y[i] = point[1]
            z[i] = point[2]
            i += 1
            
print(x[-100:-1])

# Create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Plot the values
ax.scatter3D(x[::50], y[::50], z[::50], c = 'b', marker='.')

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_xlim3d(-0.4, 0.4)
ax.set_ylim3d(-0.4, 0.4)
ax.set_zlim3d(-0.5, -0.2)

plt.show()