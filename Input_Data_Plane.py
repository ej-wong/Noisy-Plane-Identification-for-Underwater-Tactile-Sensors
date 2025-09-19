# We assume that the tactile sensor has given us the coordinates of the points it has measured on the plane.
# The coordinates given are random, obey physical constraints of the sensor, and have noise.

# steps:
# define plane
# define vector normal to plane
# RANDOMLY rotate vector about RANDOM rotation axis (Rodrigues' Formula)
# using new vector, define random line in the direction of the vector
# move m distance away from the line in RANDOM perpendicular direction and define second parallel line
# define third parallel line
# find intersection point of all three lines with the plane.
import numpy as np
import matplotlib.pyplot as plt
from Define_Plane import plane
from Define_Line import line
from noise_plane import noises

gaussian_sigma = 0.75
uniform_min = 0
uniform_max = 1.5
poisson_lambda = 0.75

### input data 3 actuators

def plane_input_3_gaussian():
    # plane to indentify
    p1_real = x1, y1, z1 = 1, -2, 7
    p2_real = x2, y2, z2 = -7, -2, -3
    p3_real = x3, y3, z3 = 4, -5, 1

    xx_real, yy_real, zz_real, n_real, d_real = plane(x1, y1, z1,
                                                        x2, y2, z2,
                                                        x3, y3, z3)

    k = 5 # distance between actuators and centre of sensor [cm]
    l = 1.2 # uncompressed actuator length [cm]
    m = k*np.sqrt(2*(1-np.cos(2*np.pi/3))) # distance between adjacent actuators [cm]
    theta_cr = np.arctan(l/(np.sqrt(3)*m/2)) # maximum angle between plane normal and sensor vector [rad]
    nu = 2*np.pi/3 # angle between actuators [rad]

    # Generate first random vector
    rand1 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate rotation axis using random unit vector
    v_r = np.cross(n_real, rand1)
    v_rotation = v_r/np.sqrt(v_r[0]**2+v_r[1]**2+v_r[2]**2) # normalized

    # Rotate unit_normal about v_rotation by random amount to obtain sensor vector
    theta_rand = np.random.uniform(low = -theta_cr, high = theta_cr, size = 1)
    v_sensor = n_real*np.cos(theta_rand) + np.cross(v_rotation, n_real)*np.sin(theta_rand) + v_rotation*(np.dot(v_rotation, n_real))*(1-np.cos(theta_rand))

    xx_line_n , yy_line_n, zz_line_n, a_n, b_n = line(0, 0, 0, n_real[0], n_real[1], n_real[2])
    xx_line_r , yy_line_r, zz_line_r, a_r, b_r = line(0, 0, 0, v_sensor[0], v_sensor[1], v_sensor[2])

    # Generate second random vector
    rand2 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate random perpendicular vector with length k
    g = np.cross(v_sensor, rand2)
    v_p = k * g / np.sqrt(g[0]**2+g[1]**2+g[2]**2)

    # Generate vector along actuator #1
    p1_act1 = v_sensor + v_p
    p2_act1 = v_sensor + p1_act1
    xx_act1, yy_act1, zz_act1, a_act1, b_act1 = line(p1_act1[0], p1_act1[1], p1_act1[2], p2_act1[0], p2_act1[1], p2_act1[2])

    # Generate vector along actuator #2
    p1_act2 = v_p*np.cos(nu) + np.cross(v_sensor, v_p)*np.sin(nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(nu))   + v_sensor
    p2_act2 = v_sensor + p1_act2
    xx_act2, yy_act2, zz_act2, a_act2, b_act2 = line(p1_act2[0], p1_act2[1], p1_act2[2], p2_act2[0], p2_act2[1], p2_act2[2])

    # Generate vector along actuator #3
    p1_act3 = v_p*np.cos(-nu) + np.cross(v_sensor, v_p)*np.sin(-nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(-nu))   + v_sensor
    p2_act3 = v_sensor + p1_act3
    xx_act3, yy_act3, zz_act3, a_act3, b_act3 = line(p1_act3[0], p1_act3[1], p1_act3[2], p2_act3[0], p2_act3[1], p2_act3[2])

    # finding actuator contact points (divide by zero error)
    ctc_1 = np.array(a_act1) + ((d_real - np.dot(a_act1, n_real)) / np.dot(b_act1, n_real)) * np.array(b_act1)
    ctc_2 = np.array(a_act2) + ((d_real - np.dot(a_act2, n_real)) / np.dot(b_act2, n_real)) * np.array(b_act2)
    ctc_3 = np.array(a_act3) + ((d_real - np.dot(a_act3, n_real)) / np.dot(b_act3, n_real)) * np.array(b_act3)

    # simulating noise normal to sensor
    noise, _, _ = noises()
  
    p1 = ctc_1 + noise[0]*v_sensor
    p2 = ctc_2 + noise[1]*v_sensor
    p3 = ctc_3 + noise[2]*v_sensor

    points = np.array([p1, p2, p3])

    # print(f"critical angle: {theta_cr}")

    # # plotting
    # fig = plt.figure()
    # plt3d = fig.add_subplot(111, projection='3d')
    # plane_plot = plt3d.plot_surface(xx_real, yy_real, zz_real, alpha = 0.5)
    # line_plot_normal = plt3d.plot(xx_line_n, yy_line_n, zz_line_n, color='k', label='Line')
    # line_plot_random = plt3d.plot(xx_line_r, yy_line_r, zz_line_r, color='r', label='Line')
    # line_plot_act1 = plt3d.plot(xx_act1, yy_act1, zz_act1, color='b', label='Line')
    # line_plot_act2 = plt3d.plot(xx_act2, yy_act2, zz_act2, color='b', label='Line')
    # line_plot_act3 = plt3d.plot(xx_act3, yy_act3, zz_act3, color='b', label='Line')
    # point_plot_act1 = plt3d.scatter(ctc_1[0], ctc_1[1], ctc_1[2], color='k', marker='o', label='Point')
    # point_plot_act2 = plt3d.scatter(ctc_2[0], ctc_2[1], ctc_2[2], color='k', marker='o', label='Point')
    # point_plot_act3 = plt3d.scatter(ctc_3[0], ctc_3[1], ctc_3[2], color='k', marker='o', label='Point')
    # point_plot_act1_noise = plt3d.scatter(p1[0], p1[1], p1[2], color='r', marker='o', label='Point')
    # point_plot_act2_noisse = plt3d.scatter(p2[0], p2[1], p2[2], color='r', marker='o', label='Point')
    # point_plot_act3_noise = plt3d.scatter(p3[0], p3[1], p3[2], color='r', marker='o', label='Point')
    # # plane_plot_fit = plt3d.plot_surface(xx_detect, yy_detect, zz_detect, color='k', alpha = 0.5)

    # plt3d.set_xlim(-10,10)
    # plt3d.set_ylim(-10,10)
    # plt3d.set_zlim(-10,10)

    # plt3d.set_xlabel('x [cm]', fontsize=10)
    # plt3d.set_ylabel('y [cm]', fontsize=10)
    # plt3d.set_zlabel('z [cm]', fontsize=10)

    # plt3d.tick_params(axis='x', labelsize=8)
    # plt3d.tick_params(axis='y', labelsize=8)
    # plt3d.tick_params(axis='z', labelsize=8)

    plt.show()

    return points, p1_real, p2_real, p3_real, n_real, d_real

def plane_input_3_uniform():
    # plane to indentify
    p1_real = x1, y1, z1 = 1, -2, 7
    p2_real = x2, y2, z2 = -7, -2, -3
    p3_real = x3, y3, z3 = 4, -5, 1

    xx_real, yy_real, zz_real, n_real, d_real = plane(x1, y1, z1,
                                                        x2, y2, z2,
                                                        x3, y3, z3)

    k = 5 # distance between actuators and centre of sensor [cm]
    l = 1.2 # uncompressed actuator length [cm]
    m = k*np.sqrt(2*(1-np.cos(2*np.pi/3))) # distance between adjacent actuators [cm]
    theta_cr = np.arctan(l/(np.sqrt(3)*m/2)) # maximum angle between plane normal and sensor vector [rad]
    nu = 2*np.pi/3 # angle between actuators [rad]

    # Generate first random vector
    rand1 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate rotation axis using random unit vector
    v_r = np.cross(n_real, rand1)
    v_rotation = v_r/np.sqrt(v_r[0]**2+v_r[1]**2+v_r[2]**2) # normalized

    # Rotate unit_normal about v_rotation by random amount to obtain sensor vector
    theta_rand = np.random.uniform(low = -theta_cr, high = theta_cr, size = 1)
    v_sensor = n_real*np.cos(theta_rand) + np.cross(v_rotation, n_real)*np.sin(theta_rand) + v_rotation*(np.dot(v_rotation, n_real))*(1-np.cos(theta_rand))

    xx_line_n , yy_line_n, zz_line_n, a_n, b_n = line(0, 0, 0, n_real[0], n_real[1], n_real[2])
    xx_line_r , yy_line_r, zz_line_r, a_r, b_r = line(0, 0, 0, v_sensor[0], v_sensor[1], v_sensor[2])

    # Generate second random vector
    rand2 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate random perpendicular vector with length k
    g = np.cross(v_sensor, rand2)
    v_p = k * g / np.sqrt(g[0]**2+g[1]**2+g[2]**2)

    # Generate vector along actuator #1
    p1_act1 = v_sensor + v_p
    p2_act1 = v_sensor + p1_act1
    xx_act1, yy_act1, zz_act1, a_act1, b_act1 = line(p1_act1[0], p1_act1[1], p1_act1[2], p2_act1[0], p2_act1[1], p2_act1[2])

    # Generate vector along actuator #2
    p1_act2 = v_p*np.cos(nu) + np.cross(v_sensor, v_p)*np.sin(nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(nu))   + v_sensor
    p2_act2 = v_sensor + p1_act2
    xx_act2, yy_act2, zz_act2, a_act2, b_act2 = line(p1_act2[0], p1_act2[1], p1_act2[2], p2_act2[0], p2_act2[1], p2_act2[2])

    # Generate vector along actuator #3
    p1_act3 = v_p*np.cos(-nu) + np.cross(v_sensor, v_p)*np.sin(-nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(-nu))   + v_sensor
    p2_act3 = v_sensor + p1_act3
    xx_act3, yy_act3, zz_act3, a_act3, b_act3 = line(p1_act3[0], p1_act3[1], p1_act3[2], p2_act3[0], p2_act3[1], p2_act3[2])

    # finding actuator contact points (divide by zero error)
    ctc_1 = np.array(a_act1) + ((d_real - np.dot(a_act1, n_real)) / np.dot(b_act1, n_real)) * np.array(b_act1)
    ctc_2 = np.array(a_act2) + ((d_real - np.dot(a_act2, n_real)) / np.dot(b_act2, n_real)) * np.array(b_act2)
    ctc_3 = np.array(a_act3) + ((d_real - np.dot(a_act3, n_real)) / np.dot(b_act3, n_real)) * np.array(b_act3)

    # simulating noise normal to sensor
    _, noise, _ = noises()
  
    p1 = ctc_1 + noise[0]*v_sensor
    p2 = ctc_2 + noise[1]*v_sensor
    p3 = ctc_3 + noise[2]*v_sensor

    points = np.array([p1, p2, p3])

    # print(f"critical angle: {theta_cr}")

    # # plotting
    # fig = plt.figure()
    # plt3d = fig.add_subplot(111, projection='3d')
    # plane_plot = plt3d.plot_surface(xx_real, yy_real, zz_real, alpha = 0.5)
    # line_plot_normal = plt3d.plot(xx_line_n, yy_line_n, zz_line_n, color='k', label='Line')
    # line_plot_random = plt3d.plot(xx_line_r, yy_line_r, zz_line_r, color='r', label='Line')
    # line_plot_act1 = plt3d.plot(xx_act1, yy_act1, zz_act1, color='b', label='Line')
    # line_plot_act2 = plt3d.plot(xx_act2, yy_act2, zz_act2, color='b', label='Line')
    # line_plot_act3 = plt3d.plot(xx_act3, yy_act3, zz_act3, color='b', label='Line')
    # point_plot_act1 = plt3d.scatter(ctc_1[0], ctc_1[1], ctc_1[2], color='k', marker='o', label='Point')
    # point_plot_act2 = plt3d.scatter(ctc_2[0], ctc_2[1], ctc_2[2], color='k', marker='o', label='Point')
    # point_plot_act3 = plt3d.scatter(ctc_3[0], ctc_3[1], ctc_3[2], color='k', marker='o', label='Point')
    # point_plot_act1_noise = plt3d.scatter(p1[0], p1[1], p1[2], color='r', marker='o', label='Point')
    # point_plot_act2_noisse = plt3d.scatter(p2[0], p2[1], p2[2], color='r', marker='o', label='Point')
    # point_plot_act3_noise = plt3d.scatter(p3[0], p3[1], p3[2], color='r', marker='o', label='Point')
    # # plane_plot_fit = plt3d.plot_surface(xx_detect, yy_detect, zz_detect, color='k', alpha = 0.5)

    # plt3d.set_xlim(-10,10)
    # plt3d.set_ylim(-10,10)
    # plt3d.set_zlim(-10,10)

    # plt3d.set_xlabel('x [cm]', fontsize=10)
    # plt3d.set_ylabel('y [cm]', fontsize=10)
    # plt3d.set_zlabel('z [cm]', fontsize=10)

    # plt3d.tick_params(axis='x', labelsize=8)
    # plt3d.tick_params(axis='y', labelsize=8)
    # plt3d.tick_params(axis='z', labelsize=8)

    plt.show()

    return points, p1_real, p2_real, p3_real, n_real, d_real

def plane_input_3_poisson():
    # plane to indentify
    p1_real = x1, y1, z1 = 1, -2, 7
    p2_real = x2, y2, z2 = -7, -2, -3
    p3_real = x3, y3, z3 = 4, -5, 1

    xx_real, yy_real, zz_real, n_real, d_real = plane(x1, y1, z1,
                                                        x2, y2, z2,
                                                        x3, y3, z3)

    k = 5 # distance between actuators and centre of sensor [cm]
    l = 1.2 # uncompressed actuator length [cm]
    m = k*np.sqrt(2*(1-np.cos(2*np.pi/3))) # distance between adjacent actuators [cm]
    theta_cr = np.arctan(l/(np.sqrt(3)*m/2)) # maximum angle between plane normal and sensor vector [rad]
    nu = 2*np.pi/3 # angle between actuators [rad]

    # Generate first random vector
    rand1 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate rotation axis using random unit vector
    v_r = np.cross(n_real, rand1)
    v_rotation = v_r/np.sqrt(v_r[0]**2+v_r[1]**2+v_r[2]**2) # normalized

    # Rotate unit_normal about v_rotation by random amount to obtain sensor vector
    theta_rand = np.random.uniform(low = -theta_cr, high = theta_cr, size = 1)
    v_sensor = n_real*np.cos(theta_rand) + np.cross(v_rotation, n_real)*np.sin(theta_rand) + v_rotation*(np.dot(v_rotation, n_real))*(1-np.cos(theta_rand))

    xx_line_n , yy_line_n, zz_line_n, a_n, b_n = line(0, 0, 0, n_real[0], n_real[1], n_real[2])
    xx_line_r , yy_line_r, zz_line_r, a_r, b_r = line(0, 0, 0, v_sensor[0], v_sensor[1], v_sensor[2])

    # Generate second random vector
    rand2 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate random perpendicular vector with length k
    g = np.cross(v_sensor, rand2)
    v_p = k * g / np.sqrt(g[0]**2+g[1]**2+g[2]**2)

    # Generate vector along actuator #1
    p1_act1 = v_sensor + v_p
    p2_act1 = v_sensor + p1_act1
    xx_act1, yy_act1, zz_act1, a_act1, b_act1 = line(p1_act1[0], p1_act1[1], p1_act1[2], p2_act1[0], p2_act1[1], p2_act1[2])

    # Generate vector along actuator #2
    p1_act2 = v_p*np.cos(nu) + np.cross(v_sensor, v_p)*np.sin(nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(nu))   + v_sensor
    p2_act2 = v_sensor + p1_act2
    xx_act2, yy_act2, zz_act2, a_act2, b_act2 = line(p1_act2[0], p1_act2[1], p1_act2[2], p2_act2[0], p2_act2[1], p2_act2[2])

    # Generate vector along actuator #3
    p1_act3 = v_p*np.cos(-nu) + np.cross(v_sensor, v_p)*np.sin(-nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(-nu))   + v_sensor
    p2_act3 = v_sensor + p1_act3
    xx_act3, yy_act3, zz_act3, a_act3, b_act3 = line(p1_act3[0], p1_act3[1], p1_act3[2], p2_act3[0], p2_act3[1], p2_act3[2])

    # finding actuator contact points (divide by zero error)
    ctc_1 = np.array(a_act1) + ((d_real - np.dot(a_act1, n_real)) / np.dot(b_act1, n_real)) * np.array(b_act1)
    ctc_2 = np.array(a_act2) + ((d_real - np.dot(a_act2, n_real)) / np.dot(b_act2, n_real)) * np.array(b_act2)
    ctc_3 = np.array(a_act3) + ((d_real - np.dot(a_act3, n_real)) / np.dot(b_act3, n_real)) * np.array(b_act3)

    # simulating noise normal to sensor
    _, _, noise = noises()
  
    p1 = ctc_1 + noise[0]*v_sensor
    p2 = ctc_2 + noise[1]*v_sensor
    p3 = ctc_3 + noise[2]*v_sensor

    points = np.array([p1, p2, p3])

    # print(f"critical angle: {theta_cr}")

    # # plotting
    # fig = plt.figure()
    # plt3d = fig.add_subplot(111, projection='3d')
    # plane_plot = plt3d.plot_surface(xx_real, yy_real, zz_real, alpha = 0.5)
    # line_plot_normal = plt3d.plot(xx_line_n, yy_line_n, zz_line_n, color='k', label='Line')
    # line_plot_random = plt3d.plot(xx_line_r, yy_line_r, zz_line_r, color='r', label='Line')
    # line_plot_act1 = plt3d.plot(xx_act1, yy_act1, zz_act1, color='b', label='Line')
    # line_plot_act2 = plt3d.plot(xx_act2, yy_act2, zz_act2, color='b', label='Line')
    # line_plot_act3 = plt3d.plot(xx_act3, yy_act3, zz_act3, color='b', label='Line')
    # point_plot_act1 = plt3d.scatter(ctc_1[0], ctc_1[1], ctc_1[2], color='k', marker='o', label='Point')
    # point_plot_act2 = plt3d.scatter(ctc_2[0], ctc_2[1], ctc_2[2], color='k', marker='o', label='Point')
    # point_plot_act3 = plt3d.scatter(ctc_3[0], ctc_3[1], ctc_3[2], color='k', marker='o', label='Point')
    # point_plot_act1_noise = plt3d.scatter(p1[0], p1[1], p1[2], color='r', marker='o', label='Point')
    # point_plot_act2_noisse = plt3d.scatter(p2[0], p2[1], p2[2], color='r', marker='o', label='Point')
    # point_plot_act3_noise = plt3d.scatter(p3[0], p3[1], p3[2], color='r', marker='o', label='Point')
    # # plane_plot_fit = plt3d.plot_surface(xx_detect, yy_detect, zz_detect, color='k', alpha = 0.5)

    # plt3d.set_xlim(-10,10)
    # plt3d.set_ylim(-10,10)
    # plt3d.set_zlim(-10,10)

    # plt3d.set_xlabel('x [cm]', fontsize=10)
    # plt3d.set_ylabel('y [cm]', fontsize=10)
    # plt3d.set_zlabel('z [cm]', fontsize=10)

    # plt3d.tick_params(axis='x', labelsize=8)
    # plt3d.tick_params(axis='y', labelsize=8)
    # plt3d.tick_params(axis='z', labelsize=8)

    plt.show()

    return points, p1_real, p2_real, p3_real, n_real, d_real

### input data 4 actuators

def plane_input_4(task):
    p1_real = x1, y1, z1 = -3, 7, 3
    p2_real = x2, y2, z2 = 6, -4, 0
    p3_real = x3, y3, z3 = -1, 0, 4

    xx_real, yy_real, zz_real, n_real, d_real = plane(x1, y1, z1,
                                                        x2, y2, z2,
                                                        x3, y3, z3)

    k = 5 # distance between actuators and centre of sensor [cm]
    l = 1.2 # uncompressed actuator length [cm]
    m = 2*k # furthest distance between 2 sensors
    theta_cr = np.arctan(l/m) # maximum angle between plane normal and sensor vector [rad]
    nu = 2*np.pi/4 # angle between actuators [rad]

    # Generate first random vector
    rand1 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate rotation axis using random unit vector
    v_r = np.cross(n_real, rand1)
    v_rotation = v_r/np.sqrt(v_r[0]**2+v_r[1]**2+v_r[2]**2) # normalized

    # Rotate unit_normal about v_rotation by random amount to obtain sensor vector
    theta_rand = np.random.uniform(low = -theta_cr, high = theta_cr, size = 1)
    v_sensor = n_real*np.cos(theta_rand) + np.cross(v_rotation, n_real)*np.sin(theta_rand) + v_rotation*(np.dot(v_rotation, n_real))*(1-np.cos(theta_rand))

    xx_line_n , yy_line_n, zz_line_n, a_n, b_n = line(0, 0, 0, n_real[0], n_real[1], n_real[2])
    xx_line_r , yy_line_r, zz_line_r, a_r, b_r = line(0, 0, 0, v_sensor[0], v_sensor[1], v_sensor[2])

    # Generate second random vector
    rand2 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate random perpendicular vector with length m
    g = np.cross(v_sensor, rand2)
    v_p = m * g / np.sqrt(g[0]**2+g[1]**2+g[2]**2)

    # Generate vector along actuator #1
    p1_act1 = v_sensor + v_p
    p2_act1 = v_sensor + p1_act1
    xx_act1, yy_act1, zz_act1, a_act1, b_act1 = line(p1_act1[0], p1_act1[1], p1_act1[2], p2_act1[0], p2_act1[1], p2_act1[2])

    # Generate vector along actuator #2
    p1_act2 = v_p*np.cos(nu) + np.cross(v_sensor, v_p)*np.sin(nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(nu)) + v_sensor
    p2_act2 = v_sensor + p1_act2
    xx_act2, yy_act2, zz_act2, a_act2, b_act2 = line(p1_act2[0], p1_act2[1], p1_act2[2], p2_act2[0], p2_act2[1], p2_act2[2])

    # Generate vector along actuator #3
    p1_act3 = v_p*np.cos(-nu) + np.cross(v_sensor, v_p)*np.sin(-nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(-nu)) + v_sensor
    p2_act3 = v_sensor + p1_act3
    xx_act3, yy_act3, zz_act3, a_act3, b_act3 = line(p1_act3[0], p1_act3[1], p1_act3[2], p2_act3[0], p2_act3[1], p2_act3[2])

    # Generate vector along actuator #4
    p1_act4 = v_p*np.cos(2*nu) + np.cross(v_sensor, v_p)*np.sin(2*nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(2*nu)) + v_sensor
    p2_act4 = v_sensor + p1_act4
    xx_act4, yy_act4, zz_act4, a_act4, b_act4 = line(p1_act4[0], p1_act4[1], p1_act4[2], p2_act4[0], p2_act4[1], p2_act4[2])

    # finding actuator contact points (divide by zero error)
    ctc_1 = np.array(a_act1) + ((d_real - np.dot(a_act1, n_real)) / np.dot(b_act1, n_real)) * np.array(b_act1)
    ctc_2 = np.array(a_act2) + ((d_real - np.dot(a_act2, n_real)) / np.dot(b_act2, n_real)) * np.array(b_act2)
    ctc_3 = np.array(a_act3) + ((d_real - np.dot(a_act3, n_real)) / np.dot(b_act3, n_real)) * np.array(b_act3)
    ctc_4 = np.array(a_act4) + ((d_real - np.dot(a_act4, n_real)) / np.dot(b_act4, n_real)) * np.array(b_act4)
    
    if task == 'gaussian':
        # Perform task 1
        noise = abs(np.random.normal(loc=0, scale=gaussian_sigma, size=4))
    elif task == 'uniform':
        # Perform task 2
        noise = np.random.uniform(low=uniform_min, high=uniform_max, size=4)
    elif task == 'poisson':
        noise = np.random.poisson(lam=poisson_lambda, size=4)
    
    p1 = ctc_1 + noise[0]*v_sensor
    p2 = ctc_2 + noise[1]*v_sensor
    p3 = ctc_3 + noise[2]*v_sensor
    p4 = ctc_4 + noise[3]*v_sensor
    points = np.array([p1, p2, p3, p4])
    
    return points, p1_real, p2_real, p3_real, n_real, d_real

### input data 5 actuators (equally spaced around circle)

def plane_input_5(task):
    p1_real = x1, y1, z1 = -3, 7, 3
    p2_real = x2, y2, z2 = 6, -4, 0
    p3_real = x3, y3, z3 = -1, 0, 4

    xx_real, yy_real, zz_real, n_real, d_real = plane(x1, y1, z1,
                                                        x2, y2, z2,
                                                        x3, y3, z3)

    k = 5 # distance between actuators and centre of sensor [cm]
    l = 1.2 # uncompressed actuator length [cm]
    m = 2*k # furthest distance between 2 sensors
    theta_cr = np.arctan(l/m) # maximum angle between plane normal and sensor vector [rad]
    nu = 2*np.pi/4 # angle between actuators [rad]
    noise_SD = 1

    # Generate first random vector
    rand1 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate rotation axis using random unit vector
    v_r = np.cross(n_real, rand1)
    v_rotation = v_r/np.sqrt(v_r[0]**2+v_r[1]**2+v_r[2]**2) # normalized

    # Rotate unit_normal about v_rotation by random amount to obtain sensor vector
    theta_rand = np.random.uniform(low = -theta_cr, high = theta_cr, size = 1)
    v_sensor = n_real*np.cos(theta_rand) + np.cross(v_rotation, n_real)*np.sin(theta_rand) + v_rotation*(np.dot(v_rotation, n_real))*(1-np.cos(theta_rand))

    xx_line_n , yy_line_n, zz_line_n, a_n, b_n = line(0, 0, 0, n_real[0], n_real[1], n_real[2])
    xx_line_r , yy_line_r, zz_line_r, a_r, b_r = line(0, 0, 0, v_sensor[0], v_sensor[1], v_sensor[2])

    # Generate second random vector
    rand2 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate random perpendicular vector with length m
    g = np.cross(v_sensor, rand2)
    v_p = m * g / np.sqrt(g[0]**2+g[1]**2+g[2]**2)

    # Generate vector along actuator #1
    p1_act1 = v_sensor + v_p
    p2_act1 = v_sensor + p1_act1
    xx_act1, yy_act1, zz_act1, a_act1, b_act1 = line(p1_act1[0], p1_act1[1], p1_act1[2], p2_act1[0], p2_act1[1], p2_act1[2])

    # Generate vector along actuator #2
    p1_act2 = v_p*np.cos(nu) + np.cross(v_sensor, v_p)*np.sin(nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(nu)) + v_sensor
    p2_act2 = v_sensor + p1_act2
    xx_act2, yy_act2, zz_act2, a_act2, b_act2 = line(p1_act2[0], p1_act2[1], p1_act2[2], p2_act2[0], p2_act2[1], p2_act2[2])

    # Generate vector along actuator #3
    p1_act3 = v_p*np.cos(-nu) + np.cross(v_sensor, v_p)*np.sin(-nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(-nu)) + v_sensor
    p2_act3 = v_sensor + p1_act3
    xx_act3, yy_act3, zz_act3, a_act3, b_act3 = line(p1_act3[0], p1_act3[1], p1_act3[2], p2_act3[0], p2_act3[1], p2_act3[2])

    # Generate vector along actuator #4
    p1_act4 = v_p*np.cos(2*nu) + np.cross(v_sensor, v_p)*np.sin(2*nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(2*nu)) + v_sensor
    p2_act4 = v_sensor + p1_act4
    xx_act4, yy_act4, zz_act4, a_act4, b_act4 = line(p1_act4[0], p1_act4[1], p1_act4[2], p2_act4[0], p2_act4[1], p2_act4[2])

    # Generate vector along actuator #5
    p1_act5 = v_p*np.cos(3*nu) + np.cross(v_sensor, v_p)*np.sin(3*nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(3*nu)) + v_sensor
    p2_act5 = v_sensor + p1_act5
    xx_act5, yy_act5, zz_act5, a_act5, b_act5 = line(p1_act5[0], p1_act5[1], p1_act5[2], p2_act5[0], p2_act5[1], p2_act5[2])

    # finding actuator contact points (divide by zero error)
    ctc_1 = np.array(a_act1) + ((d_real - np.dot(a_act1, n_real)) / np.dot(b_act1, n_real)) * np.array(b_act1)
    ctc_2 = np.array(a_act2) + ((d_real - np.dot(a_act2, n_real)) / np.dot(b_act2, n_real)) * np.array(b_act2)
    ctc_3 = np.array(a_act3) + ((d_real - np.dot(a_act3, n_real)) / np.dot(b_act3, n_real)) * np.array(b_act3)
    ctc_4 = np.array(a_act4) + ((d_real - np.dot(a_act4, n_real)) / np.dot(b_act4, n_real)) * np.array(b_act4)
    ctc_5 = np.array(a_act5) + ((d_real - np.dot(a_act5, n_real)) / np.dot(b_act5, n_real)) * np.array(b_act5)

    if task == 'gaussian':
        # Perform task 1
        noise = abs(np.random.normal(loc=0, scale=gaussian_sigma, size=5))
    elif task == 'uniform':
        # Perform task 2
        noise = np.random.uniform(low=uniform_min, high=uniform_max, size=5)
    elif task == 'poisson':
        noise = np.random.poisson(lam=poisson_lambda, size=5)
    
    p1 = ctc_1 + noise[0]*v_sensor
    p2 = ctc_2 + noise[1]*v_sensor
    p3 = ctc_3 + noise[2]*v_sensor
    p4 = ctc_4 + noise[3]*v_sensor
    p5 = ctc_5 + noise[4]*v_sensor
    points = np.array([p1, p2, p3, p4, p5])
    
    return points, p1_real, p2_real, p3_real, n_real, d_real

### input data 5 actuators (4 equally spaced around circle with 1 in the centre)

def plane_input_5_x():
    # plane to indentify
    p1_real = x1, y1, z1 = -3, 7, 9
    p2_real = x2, y2, z2 = -3, -4, 0
    p3_real = x3, y3, z3 = 7, 0, 4

    xx_real, yy_real, zz_real, n_real, d_real = plane(x1, y1, z1,
                                                        x2, y2, z2,
                                                        x3, y3, z3)

    k = 5 # distance between actuators and centre of sensor [cm]
    l = 1.2 # uncompressed actuator length [cm]
    m = 2*k # furthest distance between 2 sensors
    theta_cr = np.arctan(l/m) # maximum angle between plane normal and sensor vector [rad]
    nu = 2*np.pi/4 # angle between actuators [rad]
    noise_SD = 1

    # Generate first random vector
    rand1 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate rotation axis using random unit vector
    v_r = np.cross(n_real, rand1)
    v_rotation = v_r/np.sqrt(v_r[0]**2+v_r[1]**2+v_r[2]**2) # normalized

    # Rotate unit_normal about v_rotation by random amount to obtain sensor vector
    theta_rand = np.random.uniform(low = -theta_cr, high = theta_cr, size = 1)
    v_sensor = n_real*np.cos(theta_rand) + np.cross(v_rotation, n_real)*np.sin(theta_rand) + v_rotation*(np.dot(v_rotation, n_real))*(1-np.cos(theta_rand))

    xx_line_n , yy_line_n, zz_line_n, a_n, b_n = line(0, 0, 0, n_real[0], n_real[1], n_real[2])
    xx_line_r , yy_line_r, zz_line_r, a_r, b_r = line(0, 0, 0, v_sensor[0], v_sensor[1], v_sensor[2])

    # Generate second random vector
    rand2 = np.random.uniform(low = -1.0, high = 1.0, size = 3)

    # Generate random perpendicular vector with length m
    g = np.cross(v_sensor, rand2)
    v_p = m * g / np.sqrt(g[0]**2+g[1]**2+g[2]**2)

    # Generate vector along actuator #1
    p1_act1 = v_sensor + v_p
    p2_act1 = v_sensor + p1_act1
    xx_act1, yy_act1, zz_act1, a_act1, b_act1 = line(p1_act1[0], p1_act1[1], p1_act1[2], p2_act1[0], p2_act1[1], p2_act1[2])

    # Generate vector along actuator #2
    p1_act2 = v_p*np.cos(nu) + np.cross(v_sensor, v_p)*np.sin(nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(nu)) + v_sensor
    p2_act2 = v_sensor + p1_act2
    xx_act2, yy_act2, zz_act2, a_act2, b_act2 = line(p1_act2[0], p1_act2[1], p1_act2[2], p2_act2[0], p2_act2[1], p2_act2[2])

    # Generate vector along actuator #3
    p1_act3 = v_p*np.cos(-nu) + np.cross(v_sensor, v_p)*np.sin(-nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(-nu)) + v_sensor
    p2_act3 = v_sensor + p1_act3
    xx_act3, yy_act3, zz_act3, a_act3, b_act3 = line(p1_act3[0], p1_act3[1], p1_act3[2], p2_act3[0], p2_act3[1], p2_act3[2])

    # Generate vector along actuator #4
    p1_act4 = v_p*np.cos(2*nu) + np.cross(v_sensor, v_p)*np.sin(2*nu) + v_sensor*(np.dot(v_sensor, v_p))*(1-np.cos(2*nu)) + v_sensor
    p2_act4 = v_sensor + p1_act4
    xx_act4, yy_act4, zz_act4, a_act4, b_act4 = line(p1_act4[0], p1_act4[1], p1_act4[2], p2_act4[0], p2_act4[1], p2_act4[2])

    # Generate vector along actuator #5
    p1_act5 = v_sensor
    p2_act5 = 2*v_sensor
    xx_act5, yy_act5, zz_act5, a_act5, b_act5 = line(p1_act5[0], p1_act5[1], p1_act5[2], p2_act5[0], p2_act5[1], p2_act5[2])

    # finding actuator contact points (divide by zero error)
    ctc_1 = np.array(a_act1) + ((d_real - np.dot(a_act1, n_real)) / np.dot(b_act1, n_real)) * np.array(b_act1)
    ctc_2 = np.array(a_act2) + ((d_real - np.dot(a_act2, n_real)) / np.dot(b_act2, n_real)) * np.array(b_act2)
    ctc_3 = np.array(a_act3) + ((d_real - np.dot(a_act3, n_real)) / np.dot(b_act3, n_real)) * np.array(b_act3)
    ctc_4 = np.array(a_act4) + ((d_real - np.dot(a_act4, n_real)) / np.dot(b_act4, n_real)) * np.array(b_act4)
    ctc_5 = np.array(a_act5) + ((d_real - np.dot(a_act5, n_real)) / np.dot(b_act5, n_real)) * np.array(b_act5)

    # simulating noise normal to sensor
    noise = np.random.normal(0, noise_SD, size = 5) # creates random noise for each point
    p1 = ctc_1 + noise[0]*v_sensor
    p2 = ctc_2 + noise[1]*v_sensor
    p3 = ctc_3 + noise[2]*v_sensor
    p4 = ctc_4 + noise[3]*v_sensor
    p5 = ctc_5 + noise[4]*v_sensor

    points = np.array([p1, p2, p3, p4, p5])

    # # plotting
    # fig = plt.figure()
    # plt3d = fig.add_subplot(111, projection='3d')
    # plane_plot = plt3d.plot_surface(xx_real, yy_real, zz_real, alpha = 0.5)
    # line_plot_normal = plt3d.plot(xx_line_n, yy_line_n, zz_line_n, color='k', label='Line')
    # line_plot_random = plt3d.plot(xx_line_r, yy_line_r, zz_line_r, color='r', label='Line')
    # line_plot_act1 = plt3d.plot(xx_act1, yy_act1, zz_act1, color='b', label='Line')
    # line_plot_act2 = plt3d.plot(xx_act2, yy_act2, zz_act2, color='b', label='Line')
    # line_plot_act3 = plt3d.plot(xx_act3, yy_act3, zz_act3, color='b', label='Line')
    # line_plot_act4 = plt3d.plot(xx_act4, yy_act4, zz_act4, color='b', label='Line')
    # line_plot_act5 = plt3d.plot(xx_act5, yy_act5, zz_act5, color='b', label='Line')
    # point_plot_act1 = plt3d.scatter(ctc_1[0], ctc_1[1], ctc_1[2], color='k', marker='o', label='Point')
    # point_plot_act2 = plt3d.scatter(ctc_2[0], ctc_2[1], ctc_2[2], color='k', marker='o', label='Point')
    # point_plot_act3 = plt3d.scatter(ctc_3[0], ctc_3[1], ctc_3[2], color='k', marker='o', label='Point')
    # point_plot_act4 = plt3d.scatter(ctc_4[0], ctc_4[1], ctc_4[2], color='k', marker='o', label='Point')
    # point_plot_act5 = plt3d.scatter(ctc_5[0], ctc_5[1], ctc_5[2], color='k', marker='o', label='Point')
    # point_plot_act1_noise = plt3d.scatter(p1[0], p1[1], p1[2], color='r', marker='o', label='Point')
    # point_plot_act2_noisse = plt3d.scatter(p2[0], p2[1], p2[2], color='r', marker='o', label='Point')
    # point_plot_act3_noise = plt3d.scatter(p3[0], p3[1], p3[2], color='r', marker='o', label='Point')
    # point_plot_act4_noise = plt3d.scatter(p4[0], p4[1], p4[2], color='r', marker='o', label='Point')
    # point_plot_act5_noise = plt3d.scatter(p5[0], p5[1], p5[2], color='r', marker='o', label='Point')

    # plt3d.set_xlim(-10,10)
    # plt3d.set_ylim(-10,10)
    # plt3d.set_zlim(-10,10)

    # plt3d.set_xlabel('X Axis')
    # plt3d.set_ylabel('Y Axis')
    # plt3d.set_zlabel('Z Axis')

    # plt3d.set_xlabel('X Axis', fontsize=8)
    # plt3d.set_ylabel('Y Axis', fontsize=8)
    # plt3d.set_zlabel('Z Axis', fontsize=8)

    # plt3d.tick_params(axis='x', labelsize=6)
    # plt3d.tick_params(axis='y', labelsize=6)
    # plt3d.tick_params(axis='z', labelsize=6)

    plt.show()

    return points, p1_real, p2_real, p3_real, n_real, d_real