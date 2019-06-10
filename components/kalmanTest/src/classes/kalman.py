import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from filterpy.stats import plot_covariance_ellipse, plot_covariance
from scipy.linalg import block_diag
# import faulthandler
# faulthandler.enable()
import matplotlib.pyplot as plt


class KalmanTracker(KalmanFilter):
    def __init__(self, x=0, y=0):
        super(KalmanTracker, self).__init__(dim_x=4, dim_z=2)

        self.init_to_position(x, y)

    def init_to_position(self, x, y):
        # Measurement Function
        self.H = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])

        # Measurement Noise Matrix
        self.R = np.eye(2) * 0.5  # 0.1 meters error
        # Process Noise Matrix
        q = Q_discrete_white_noise(dim=2, dt=1, var=0.01)
        self.Q = block_diag(q, q)

        # Initial Position X, vel X, Y, vel Y
        self.x = np.array([[x, 0, y, 0]]).T

        # Covariance Matrix
        self.P = np.eye(4) * 0.5

    def predict_with_time_diff(self, dt):
        self.F = np.array([[1, dt, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, dt],
                           [0, 0, 0, 1]])

        # # Process Noise Matrix
        # q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01)
        # self.Q = block_diag(q, q)

        self.predict()

        return self.x, self.P

    def update(self, z):
        super(KalmanTracker, self).update(z)
        return self.x, self.P


# def plotResults(measurements, predictions, covariances):
#     for pos_vel, cov_matrix in zip(predictions, covariances):
#         cov = np.array([[cov_matrix[0, 0], cov_matrix[2, 0]], [cov_matrix[0, 2], cov_matrix[2, 2]]])
#         # cov = np.array([[cov_matrix[1, 1], cov_matrix[3, 1]],[cov_matrix[1, 3], cov_matrix[3, 3]]])
#
#         [x_pos, x_vel, y_pos, y_vel] = pos_vel[:, 0]
#         mean = (x_pos, y_pos)
#         # Plotting elipses
#         plot_covariance_ellipse(mean, cov=cov, fc='r', alpha=0.20, show_semiaxis=True)
#
#         # plotting velocity
#         plt.quiver(x_pos, y_pos, x_vel, y_vel, color='b', scale_units='xy', scale=1, alpha=0.50, width=0.005)
#
#     # Plotting meditions
#     x = []
#     y = []
#     for measure in measurements:
#         if not measure: continue
#         x.append(measure[0])
#         y.append(measure[1])
#
#     plt.plot(x, y, '.g', lw=1, ls='--')
#
#     plt.axis('equal')
#     plt.show()


if __name__ == '__main__':

    a = KalmanTracker(0, 0)

    # lineal example
    # zs = ([0,0],[1,1],[2,2],[3,3],[4,4],[5,5],None,None,None,None,[10,10],None,None,[13,13],[14,14],[15,15]) #Andando en diagonal
    # Curve Example
    # zs = ([0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 4], [6, 4], [7, 4], None, None, None, [13,3],[14,2],[15,1],[16,0], None, None)
    #
    zs = [] #Curva sinusoidal
    x = np.arange(0, 20, 0.5)
    for xn in x: zs.append([xn,np.sin(xn)])
    zs.append(None)
    zs.append(None)
    zs.append(None)

    # zs = []
    # x = np.arange(0, 10, 1)
    # for xn in x:
    #     zs.append([xn, xn])
    #
    # zs.append(None)
    # zs.append(None)
    # zs.append(None)
    #
    # x = np.arange(16, 30, 1)
    # for xn in x:
    #     zs.append([xn, 15])
    #
    # zs.append(None)
    # zs.append(None)
    # zs.append(None)
    # zs.append(None)
    # zs.append(None)
    # zs.append(None)

    last_time = 0  # tiempo en el que se tomo la ultima medida

    predictions = []
    covariances = []
    for index, z in enumerate(zs):

        if index == 0: continue  # la primera medicion la usamos para inizializar el tracker

        print (".........Medicion......... ", z)
        dt = 1
        last_time = index
        predX, predP = a.predict_with_time_diff(dt)
        # predX, predP = person_tracker.predict(dt)
        predictions.append(predX)
        covariances.append(predP)

        print "Predicha", predX[0], predX[2], "Con velocidad", predX[1], predX[3]

        # Si no hay medicion no actualizo el filtro
        if not z:
            continue

        upX, upP = a.update(z)
        print "Adaptada", upX[0], upX[2], "Con velocidad", upX[1], upX[3]
        print "Siguiente probabilidad", np.diag(upP)

        print ("----------------------------")

    plotResults(zs, predictions, covariances)
