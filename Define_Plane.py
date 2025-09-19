import numpy as np
import matplotlib.pyplot as plt

def plane(x0, y0, z0,
          x1, y1, z1,
          x2, y2, z2):
    
    # Defining Plane Vectors
    u = [x1-x0, y1-y0, z1-z0]
    v = [x2-x0, y2-y0, z2-z0]

    u_cross_v = w1, w2, w3 = np.cross(u, v)

    if np.all(u_cross_v == 0):
        print("Error. The number of solution planes are infinite due to collinearity of points.")
    else:
        unit_normal = n1, n2, n3 = np.array(u_cross_v/np.sqrt(w1**2 + w2**2 + w3**2))
        a = [x0, y0, z0] # point on plane
        d = np.dot(a, unit_normal) # shortest distance from origin to plane

        if n3 == 0 and n1 == 0:
            xx, zz = np.meshgrid(range(-10,11), range(-10,11))
            yy = np.full_like(zz, d/n2) # creates copy of matrix zz and fills it with the constant y value.
        elif n3 == 0 and n2 == 0:
            yy, zz = np.meshgrid(range(-10,11), range(-10,11))
            xx = np.full_like(zz, d/n1) # creates copy of matrix yy and fills it with the constant x value.
        elif n3 == 0:
            xx, zz = np.meshgrid(range(-10,11), range(-10,11))
            yy = (d-n1*xx)/n2
        else:
            xx, yy = np.meshgrid(range(-10,11), range(-10,11))
            zz = (d - n1*xx - n2*yy) / n3
    
    return xx, yy, zz, unit_normal, d


## Errors: 
# 1. unit normal is a zero vector 
# 2. points lie in a straight line

# xx_plane, yy_plane, zz_plane, n_plane, d = plane(8,-9,2,
#                                                 3,9,9,
#                                                 8,1,2)
# fig = plt.figure()
# plt3d = fig.add_subplot(111, projection='3d')
# plane_plot = plt3d.plot_surface(xx_plane, yy_plane, zz_plane, alpha = 0.5)

# # plt3d.set_xlim(-10,10)
# # plt3d.set_ylim(-10,10)
# # plt3d.set_zlim(-10,10)

# plt3d.set_xlabel('X Axis')
# plt3d.set_ylabel('Y Axis')
# plt3d.set_zlabel('Z Axis')

# print(xx_plane)

# plt.show()

def plane_normal(n1, n2, n3, d):

    unit_normal = np.array([n1, n2, n3])

    if n3 == 0 and n1 == 0:
        xx, zz = np.meshgrid(range(-10,10), range(-10,10))
        yy = np.full_like(zz, d/n2) # creates copy of matrix zz and fills it with the constant y value.
    elif n3 == 0 and n2 == 0:
        yy, zz = np.meshgrid(range(-10,10), range(-10,10))
        xx = np.full_like(zz, d/n1) # creates copy of matrix yy and fills it with the constant x value.
    elif n3 == 0:
        xx, zz = np.meshgrid(range(-10,10), range(-10,10))
        yy = (d-n1*xx)/n2
    else:
        xx, yy = np.meshgrid(range(-10,10), range(-10,10))
        zz = (d - n1*xx - n2*yy) / n3

    return xx, yy, zz, unit_normal, d