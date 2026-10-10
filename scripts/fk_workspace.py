import _bootstrap  # noqa: F401  (puts src/ on sys.path)

from lib.calculateFK import FK

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    [-2.8973,  2.8973],
    [-1.7628,  1.7628],
    [ -2.8973,  2.8973],
    [ -3.0718,  -0.0698],
    [ -2.8973,  2.8973],
    [ -0.0175, 3.7525],
    [ -2.8973,  2.8973]
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html


samples = 1000

pts = []
    
for i in range(samples):
    angles = [np.random.uniform(minn, maxx) for (minn, maxx) in limits]
        
    jp, T0e = FK.forward(fk, angles)
        
    pos = T0e[0:3,-1]
    
     
    pts.append(pos)


pts = np.array(pts)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(pts[:,0], pts[:,1], pts[:,2])

# TODO: update this with real results
ax.scatter(0.0825,0,0.649) # plot the point (1,1,1)

ax.set_xlabel('X0')
ax.set_ylabel('Y0')
ax.set_zlabel('Z0')

plt.show()
