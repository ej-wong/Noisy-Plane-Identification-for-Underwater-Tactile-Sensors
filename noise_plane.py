import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def noises():
    gaussian_sigma = 0.75
    uniform_min = 0
    uniform_max = 1.5
    poisson_lambda = 0.75

    gaussian_noise = abs(np.random.normal(loc=0, scale=gaussian_sigma, size=3))
    uniform_noise = np.random.uniform(low=uniform_min, high=uniform_max, size=3)
    poisson_noise = np.random.poisson(lam=poisson_lambda, size=3)

    return gaussian_noise, uniform_noise, poisson_noise

# # Define grid
# x = np.linspace(-5, 5, 100)
# y = np.linspace(-5, 5, 100)
# X, Y = np.meshgrid(x, y)

# # Define desired distribution parameters
# gaussian_sigma = 0.75
# uniform_min = 0
# uniform_max = 1.5
# poisson_lambda = 0.75

# # Generate noise distributions
# gaussian_noise = np.random.normal(loc=0, scale=gaussian_sigma, size=X.shape)
# uniform_noise = np.random.uniform(low=0, high=1.5, size=X.shape)
# poisson_noise = np.random.poisson(lam=poisson_lambda, size=X.shape)

# # Plot Gaussian noise
# fig = plt.figure(figsize=(15, 5))

# # Plot for Gaussian noise
# ax1 = fig.add_subplot(131, projection='3d')
# ax1.plot_surface(X, Y, np.abs(gaussian_noise), cmap='viridis')  # Taking absolute values to ensure positivity
# ax1.set_title('Gaussian Noise (σ = 0.75)')
# ax1.set_xlabel('x [cm]')
# ax1.set_ylabel('y [cm]')
# ax1.set_zlabel('z [cm]')

# # Plot for Uniform noise
# ax2 = fig.add_subplot(132, projection='3d')
# ax2.plot_surface(X, Y, uniform_noise, cmap='viridis')
# ax2.set_title('Uniform Noise (min = 0, max = 1.5)')
# ax2.set_xlabel('x [cm]')
# ax2.set_ylabel('y [cm]')
# ax2.set_zlabel('z [cm]')

# # Plot for Poisson noise
# ax3 = fig.add_subplot(133, projection='3d')
# ax3.plot_surface(X, Y, poisson_noise, cmap='viridis')
# ax3.set_title('Poisson Noise (λ = 0.75)')
# ax3.set_xlabel('x [cm]')
# ax3.set_ylabel('y [cm]')
# ax3.set_zlabel('z [cm]')

# plt.show()