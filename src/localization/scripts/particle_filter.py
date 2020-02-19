#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import rospy

from std_msgs.msg import String
from filterpy.monte_carlo import systematic_resample
from scipy.stats import norm
from numpy.random import uniform, randn, random, seed

# initialize global
global error_val
error_val = 10000

# Particle Filter based on the following repository:
# https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
# axis origin on top left map vertex: x points left, y points down
# wall vertices are [0,0], [0,L1], [L2,0]

class ParticleFilter(object):
    def __init__(self, N, sensor_locations=None, sensor_max=None,
                 wall_lengths=[20,20], sensor_std_error=.05,
                 initial_x=None):
        
        # initialize attributes
        self.sensor_locations = sensor_locations
        self.sensor_max = sensor_max
        self.sensor_std_error = sensor_std_error
        self.wall_lengths = wall_lengths

        # initialize particles and weights
        if initial_x is not None:
            self.particles = self.create_gaussian_particles(
                mean=initial_x, std=(0.5, 0.5, np.pi/4), N=N)
        else:
            L1 = wall_lengths[0]
            L2 = wall_lengths[1]
            self.particles = self.create_uniform_particles((0,-L2), (0,L1), (-3.14159, 3.14159), N)
        
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

    def predict(self, u, std, dt=0.2):
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
            pos = self.particles[i]
            for j, sensor_loc in enumerate(self.sensor_locations):
                # find estimated distance from sensors
                dist = self.find_sensor_dist(pos, sensor_loc)

                # find new posterior probability
                self.weights[i] *= norm(dist, self.sensor_std_error).pdf(z[j])

        self.weights += 1.e-300      # avoid round-off to zero
        self.weights /= sum(self.weights) # normalize

    def estimate(self):
        """returns mean and variance of the weighted particles"""

        pos = self.particles[:, 0:3]
        mean = np.average(pos, weights=self.weights, axis=0)
        var  = np.average((pos - mean)**2, weights=self.weights, axis=0)

        # max_ind = np.argmax(self.weights)
        # best_est = self.particles[max_ind]

        return [mean, var]
    
    def resample_simple(self):
        N = len(self.particles)
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1. # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, random(N))

        # resample according to indexes
        self.particles[:] = self.particles[indexes]
        self.weights.fill(1.0 / N)

    def resample_syst(self):
        indexes = systematic_resample(self.weights)
        self.resample_from_index(indexes)
        
    def resample_from_index(self, indexes):
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / len(self.weights))

    def neff(self):
        return 1. / np.sum(np.square(self.weights))  

    def update_step(self, z, u, u_std=[.1,.05], plot_points=False):
        N = len(self.particles)

        # predict step
        self.predict(u=u, std=(.1, .05))
        
        # update step
        self.update(z=z)
        
        # resample if too few effective particles
        if self.neff() < N/2:
            self.resample_simple()
            # self.resample_syst()

        # estimate mean and variance
        est = self.estimate()
        best_est = est[0]

        if plot_points == True:
            # plot particles and mean
            plt.scatter(self.particles[:, 0], self.particles[:, 1], 
                            color='k', marker=',', s=1)

        plt.scatter(best_est[0], best_est[1], marker='s', color='r')
        return best_est

    # returns scalar projection of vec1 onto vec2
    def vector_project(self, vec1, vec2):
        return np.dot(vec1, vec2) / np.linalg.norm(vec2)

    # returns cos(angle) between vec1 and vec2
    def vector_angle(self, vec1, vec2):
        return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))

    # returns distance from sensor to walls given robot position and sensor location
    def find_sensor_dist(self, pos, sensor_loc):
        global error_val

        # extract values from pos
        x = pos[0]
        y = pos[1]
        th = pos[2]
        
        # find split, min, and max angles
        L1 = self.wall_lengths[0]
        L2 = self.wall_lengths[1]
        split_ang = np.arctan2(-y,-x)
        max_ang = np.arctan2(L1-y,-x)
        min_ang = np.arctan2(-y,-L2-x)
        
        # initialize rotation matrix
        R = np.array([[np.cos(th), -np.sin(th)],
                      [np.sin(th), np.cos(th)]])

        # rotate sensor_loc 
        sensor_loc = R.dot(np.array(sensor_loc))

        # determine measured distance
        dist = None           
        sensor_ang = np.arctan2(sensor_loc[1], sensor_loc[0])
        if (sensor_ang <= min_ang or sensor_ang >= max_ang):
            dist = error_val
        elif (sensor_ang > split_ang):
            cos_ang = self.vector_angle(sensor_loc, np.array([x,0]))
            dist = x / cos_ang
        else:
            cos_ang = self.vector_angle(sensor_loc, np.array([0, -y]))
            dist = y / cos_ang

        return dist

if __name__ == '__main__':
    global error

    seed(2)
    plt.figure()

    N = 100
    wall_lengths = [1.524, 0.9144]
    sensor_locations = ([0,1], [1,0], [0,-1])
    sensor_max = [50, 50, 50]
    sensor_readings = [error_val, 0.5, 0.6]
    initial_x = None
    u_std = [0.1, 0.05]

    pf = ParticleFilter(N, wall_lengths=wall_lengths, sensor_locations=sensor_locations, 
                        sensor_max=sensor_max, initial_x=initial_x)

    for iter in range(20):
        best_est = pf.update_step(sensor_readings, (0,0), u_std=u_std)
        print(best_est)

    d1 = pf.find_sensor_dist(best_est, sensor_locations[0])
    d2 = pf.find_sensor_dist(best_est, sensor_locations[1])
    d3 = pf.find_sensor_dist(best_est, sensor_locations[2])
    print("validation dists: %f %f %f" %(d1, d2, d3))

    plt.show()