#!/usr/bin/env python
from PyQt5 import QtWidgets, QtCore, uic
from pyqtgraph import PlotWidget, plot
from particle_filter import ParticleFilter
from geometry_msgs.msg import Twist

import rospy
import pyqtgraph as pg
import numpy as np
import sys  # We need sys so that we can pass argv to QApplication
import os

class DemoWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(DemoWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.graphWidget.getViewBox().invertY(True)
        self.graphWidget.getViewBox().invertX(True)

        self.setCentralWidget(self.graphWidget)
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setXRange(-2, 0)
        self.graphWidget.setYRange(0, 2)

        # initialize attributes
        self.u = [0,0]
        self.dt = 0.1 # 10 hz updates
        self.state = [-0.5, 1.5, 0]
        self.est_state = self.state

        # initialize filter
        N = 100
        wall_lengths = [2, 2]
        self.sensor_locations = ([0,-1], [1,-1], [1,0], [1,1], [0,1])
        self.sensor_readings = [0, 0, 0, 0, 0]
        sensor_max = [50, 50, 50, 50, 50]
        initial_x = [-0.5, 1.5, 0]

        self.filter = ParticleFilter(N, sensor_locations=self.sensor_locations, sensor_max=sensor_max,
                                     wall_lengths=wall_lengths, initial_x=initial_x)

        # initialize ROS
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.callback)

        # setup timer to run particle filter at 5 Hz
        self.timer = QtCore.QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()

        # plot data: x, y values
        pen1 = pg.mkPen(color='b')
        pen2 = pg.mkPen(color='r')
        self.pos_plot = self.graphWidget.plot([], [], pen=pen2, symbol='o', symbolSize=10, symbolBrush=('r'))
        self.est_plot = self.graphWidget.plot([], [], pen=pen1, symbol='o', symbolSize=10, symbolBrush=('b'))

    def timer_callback(self):
        u_std = [0.1, 0.05]
        self.est_state = self.filter.update_step(self.sensor_readings, self.u, u_std=u_std)

    def callback(self, data):
        self.u = [data.linear.x, data.angular.z]
        self.update_state()
        self.plot_state()
        self.find_sensor_vals()

    def find_sensor_vals(self):
        dists = []
        for sensor_loc in self.sensor_locations:
            dist = self.filter.find_sensor_dist(self.state, sensor_loc)
            dists.append(dist)

        self.sensor_readings = dists

        rospy.loginfo(dists)

    def plot_state(self):
        r = 0.1

        x_pos = [self.state[0], self.state[0] + r * np.cos(self.state[2])]
        y_pos = [self.state[1], self.state[1] + r * np.sin(self.state[2])]
        self.pos_plot.setData(x_pos, y_pos)

        x_est = [self.est_state[0], self.est_state[0] + r * np.cos(self.est_state[2])]
        y_est = [self.est_state[1], self.est_state[1] + r * np.sin(self.est_state[2])]
        self.est_plot.setData(x_est, y_est)

    # update state w/ runge-kutta dead reckoning
    def update_state(self):
        v = self.u[0]
        w = self.u[1]
        
        x = self.state[0]
        y = self.state[1]
        th = self.state[2]

        k00 = v * np.cos(th)
        k01 = v * np.sin(th)
        k02 = w
        
        k10 = v * np.cos(th + (self.dt/2) * k02)
        k11 = v * np.sin(th + (self.dt/2) * k02)
        k12 = w

        k20 = v * np.cos(th + (self.dt/2) * k12)
        k21 = v * np.sin(th + (self.dt/2) * k12)
        k22 = w

        k30 = v * np.cos(th + (self.dt) * k22)
        k31 = v * np.sin(th + (self.dt) * k22)
        k32 = w

        self.state[0] = x + (self.dt / 6) * (k00 + 2*(k10 + k20) + k30)
        self.state[1] = y + (self.dt / 6) * (k01 + 2*(k11 + k21) + k31)
        self.state[2] = th + (self.dt / 6) * (k02 + 2*(k12 + k22) + k32)

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = DemoWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()