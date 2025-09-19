import numpy as np
import matplotlib.pyplot as plt

def line(a_x, a_y, a_z, 
         b_x, b_y, b_z):

     a = [a_x, a_y, a_z]
     b = [b_x-a_x, b_y-a_y, b_z-a_z]
    
     lmda = np.linspace(-10, 10, 100)
     xx_line = a_x + lmda*(b[0])
     yy_line = a_y + lmda*(b[1])
     zz_line = a_z + lmda*(b[2])

     return xx_line, yy_line, zz_line, a, b


# xx_line, yy_line, zz_line, a_line, b_line = line(2,2,2,
#                                                 1,1,4)

# fig = plt.figure()
# plt3d = fig.add_subplot(111, projection='3d')
# line_plot = plt3d.plot(xx_line, yy_line, zz_line, color='k', label='Line')

# plt3d.set_xlim(-10,10)
# plt3d.set_ylim(-10,10)
# plt3d.set_zlim(-10,10)

# plt3d.set_xlabel('X Axis')
# plt3d.set_ylabel('Y Axis')
# plt3d.set_zlabel('Z Axis')

# plt.show()