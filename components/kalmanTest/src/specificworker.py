#
# Copyright (C) 2019 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import datetime
from Queue import Queue
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from filterpy.stats import plot_covariance
from genericworker import *
from classes.kalman_acceleration import KalmanTracker


class dataToRepresent():
    def __init__(self):
        self.prediction = None
        self.covariance = None
        self.measurement = None


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 1000
        self.tracker = None
        self._last_time_detected = -1
        self._last_time_predicted = -1

        self.kalman_initialized = False
        self._detection_queue = Queue()

        self._data_queue = deque(maxlen=4)

        self.timer.start(self.Period)

    def __del__(self):
        print 'SpecificWorker destructor'

    def setParams(self, params):
        return True

    def plotResults(self):
        plt.clf()
        plt.ion()


        for d in self._data_queue:
            pos_vel = d.prediction
            cov_matrix = d.covariance
            measure = d.measurement

            if cov_matrix is None: return

            cov = np.array([[cov_matrix[0, 0], cov_matrix[3, 0]], [cov_matrix[0, 3], cov_matrix[3, 3]]])
            [x_pos, x_vel, x_ac, y_pos, y_vel, y_ac] = pos_vel[:, 0]
            # cov = np.array([[cov_matrix[0, 0], cov_matrix[2, 0]], [cov_matrix[0, 2], cov_matrix[2, 2]]])
            # [x_pos, x_vel, y_pos, y_vel] = pos_vel[:, 0]
            mean = (x_pos, y_pos)

            plot_covariance(mean, cov=cov, fc='r', alpha=0.2)

            if measure is not None:

                plt.plot(measure[0], measure[1], '.g', lw=1, ls='--')

        plt.quiver(x_pos, y_pos, x_vel, y_vel, color='b', scale_units='xy', scale=1, alpha=0.8, width=0.0035)
        # 	# # plotting acceleration
        plt.quiver(x_pos, y_pos, x_ac, y_ac, color='y', scale_units='xy', scale=1, alpha=0.8, width=0.0035)

        # plt.show()

        plt.xlim([-10,10])
        plt.ylim([-10, 10])
        plt.pause(0.0001)

        plt.show()

    @QtCore.Slot()
    def compute(self):

        data = dataToRepresent()
        if self.kalman_initialized:

            current_time = datetime.datetime.now()
            dt = (current_time - self._last_time_predicted).total_seconds()
            predicted, cov = self.tracker.predict_with_time_diff(dt)
            self._last_time_predicted = current_time

            print "Dato predicho", predicted[0], predicted[2]
            data.prediction = predicted
            data.covariance = cov

        if (self._detection_queue.empty() == False):
            humansFromCam = self._detection_queue.get_nowait()
            position = humansFromCam.humanList[0].pos
            detection_time = datetime.datetime.now()
            self._last_time_detected = detection_time
            self._last_time_predicted = detection_time

            if (self.kalman_initialized == False):
                print "Iniciamos kalman en ", position.x / 1000, position.z / 1000
                self.tracker = KalmanTracker(position.x / 1000, position.z / 1000)
                self.kalman_initialized = True

            else:
                print "Dato recibido", position.x / 1000, position.z / 1000
                data.measurement = [position.x / 1000, position.z / 1000]
                predicted, cov = self.tracker.update([position.x / 1000, position.z / 1000])

        else:
            data.measurement = None

        if (data.covariance is not None):
            self._data_queue.append(data)



            self.plotResults()


            # raw_input('Press Enter')

    #
    # obtainHumanPose
    #
    def obtainHumanPose(self, humansFromCam):
        self._detection_queue.put(humansFromCam)
