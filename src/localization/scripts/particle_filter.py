import matplotlib.pyplot as plt
import numpy as np

from filterpy.monte_carlo import systematic_resample
from scipy.stats import norm
from numpy.random import uniform, randn

# globals
error_val = 100

# Particle Filter based on the following repository:
# https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
# axis origin on top left map vertex: x points left, y points down
# wall vertices are [0,0], [0,L1], [L2,0]

class ParticleFilter(object):
    def __init__(self, N, sensor_locations=None, sensor_std_error=.1,
                 initial_x=None):

        # initialize attributes
        self.sensor_locations = sensor_locations
        self.sensor_std_error = sensor_std_error

        # initialize particles and weights
        if initial_x is not None:
            self.particles = self.create_gaussian_particles(
                mean=initial_x, std=(0.5, 0.5, np.pi/4), N=N)
        else:
            self.particles = self.create_uniform_particles((0,20), (0,20), (-3.14159, 3.14159), N)
        
        self.weights = np.ones(N) / N
        

    def create_uniform_particles(self, x_range, y_range, hdg_range, N):
        particles = np.empty((N, 3))
        particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
        particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
        particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N)
        particles[:, 2] = np.arctan2(np.sin(particles[:, 2]), np.cos(particles[:, 2]))
        return particles

    def create_gaussian_particles(self, mean, std, N):
        particles = np.empty((N, 3))
        particles[:, 0] = mean[0] + (randn(N) * std[0])
        particles[:, 1] = mean[1] + (randn(N) * std[1])
        particles[:, 2] = mean[2] + (randn(N) * std[2])
        particles[:, 2] = np.arctan2(np.sin(particles[:, 2]), np.cos(particles[:, 2]))
        return particles

    def predict(self, u, std, dt=1.):
        """ move according to control input u (heading change, velocity)
        with noise Q (std heading change, std velocity)`"""

        N = len(self.particles)

        # update heading
        self.particles[:, 2] += u[0] + (randn(N) * std[0])
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2]))

        # move in the (noisy) commanded direction
        dist = (u[1] * dt) + (randn(N) * std[1])
        self.particles[:, 0] += np.cos(self.particles[:, 2]) * dist
        self.particles[:, 1] += np.sin(self.particles[:, 2]) * dist

    def update(self, z):
        global error_val
        
        N = len(self.particles)

        for i in range(N):
            # get particle x, y, th
            particle = self.particles[i]
            x = self.particles[i,0]
            y = self.particles[i,1]
            th = self.particles[i,2]

            for j, sensor_loc in enumerate(self.sensor_locations):
                # initialize rotation matrix
                R = np.array([[np.cos(th), -np.sin(th)],
                              [np.sin(th), np.cos(th)]])

                # rotate sensor_loc 
                sensor_loc = R.dot(np.array(sensor_loc))

                # project onto walls (check to ensure sensor will hit wall)
                proj_wall1 = self.project_vector(np.array([x, 0]), sensor_loc)
                wall1_dist = np.average(proj_wall1 / sensor_loc) 
                if wall1_dist < 0:
                    wall1_dist = error_val

                proj_wall2 = self.project_vector(np.array([0, -y]), sensor_loc)
                wall2_dist = np.average(proj_wall2 / sensor_loc) 
                if wall2_dist < 0:
                    wall2_dist = error_val

                # find new posterior probability
                # distance = np.array([[wall1_dist], [wall2_dist]])
                self.weights[i] *= norm(wall1_dist, self.sensor_std_error).pdf(z[j])
                self.weights[i] *= norm(wall2_dist, self.sensor_std_error).pdf(z[j])

        self.weights += 1.e-300      # avoid round-off to zero
        self.weights /= sum(self.weights) # normalize

    def estimate(self):
        """returns mean and variance of the weighted particles"""

        pos = self.particles[:, 0:2]
        mean = np.average(pos, weights=self.weights, axis=0)
        var  = np.average((pos - mean)**2, weights=self.weights, axis=0)
        return mean, var
    
    def simple_resample(self):
        N = len(self.particles)
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1. # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, random(N))

        # resample according to indexes
        self.particles[:] = self.particles[indexes]
        self.weights.fill(1.0 / N)

        
    def resample_from_index(self, indexes):
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / len(self.weights))
    
    # projects vec1 onto vec2
    def project_vector(self, vec1, vec2):
        return vec2 * np.dot(vec1, vec2) / np.dot(vec2, vec2)

    def neff(self):
        return 1. / np.sum(np.square(self.weights))  

    def update_step(self, z, u):
        N = len(self.particles)

        # predict step
        print("predict")
        self.predict(u=u, std=(.2, .05))
        
        # update step
        print("update")
        self.update(z=z)
        
        # resample if too few effective particles
        if self.neff() < N/2:
            indexes = systematic_resample(self.weights)
            self.resample_from_index(indexes)
            assert np.allclose(self.weights, 1/N)

        # estimate mean and variance
        mu, var = self.estimate()
        return mu, var

if __name__ == '__main__':
    global error_val

    sensor_locations = ([0,1], [1,0], [0,-1])
    sensor_readings = [error_val, 5, 5]

    pf = ParticleFilter(5000, sensor_locations=sensor_locations)

    for iter in range(10):
      mu, var = pf.update_step(sensor_readings, (0,0))
      print(mu, var)