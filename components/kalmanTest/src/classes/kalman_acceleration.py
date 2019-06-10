import numpy as np
from filterpy.common import Q_discrete_white_noise, pprint
from filterpy.kalman import KalmanFilter
from filterpy.stats import plot_covariance_ellipse, plot_covariance, plot_gaussian_pdf
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
import scipy.linalg as linalg
import math


class KalmanTracker(KalmanFilter):
    def __init__(self, x=0, y=0):
        super(KalmanTracker, self).__init__(dim_x=6, dim_z=2)
        # Measurement Function
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0]])

        # Measurement Noise Matrix
        self.R = np.eye(2) * 0.25  # 0.5 meters2 error

        # Process Noise Matrix
        # q = Q_discrete_white_noise(dim=3, dt=1, var=0.01)
        # self.Q = block_diag(q, q)


        # Initial Position X, vel X, ac X, Y, vel Y, ac Y
        self.x = np.array([[x, 0, 0, y, 0, 0]]).T

        # Covariance Matrix
        self.P = self.P * 2.5

    def predict_with_time_diff(self, dt):
        self.F = np.array([[1., dt, .5 * dt * dt, 0., 0., 0.],
                           [0., 1., dt, 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.],
                           [0., 0., 0., 1., dt, .5 * dt * dt],
                           [0., 0., 0., 0., 1., dt],
                           [0., 0., 0., 0., 0., 1.]])

        # # Process Noise Matrix
        q = Q_discrete_white_noise(dim=3, dt=dt, var=0.01)
        self.Q = block_diag(q, q)

        self.predict()

        return self.x, self.P

    def update(self, z):
        super(KalmanTracker, self).update(z)
        return self.x, self.P


def plotResults(measurements, predictions, covariances):
    for pos_vel, cov_matrix in zip(predictions, covariances):
        # cov = np.array([[cov_matrix[1, 1], cov_matrix[4, 1]], [cov_matrix[1, 4], cov_matrix[4, 4]]])  # cov de vx con vy
        cov = np.array([[cov_matrix[0, 0], cov_matrix[3, 0]],[cov_matrix[0, 3], cov_matrix[3, 3]]]) #cov de x con y

        # cov1 = np.array([[cov_matrix[0, 0], cov_matrix[0, 1]],[cov_matrix[1, 0], cov_matrix[1, 1]]]) #cov de x con vx
        # cov2 = np.array([[cov_matrix[3, 3], cov_matrix[4, 3]],[cov_matrix[3, 4], cov_matrix[4, 4]]]) #cov de y con vy

        [x_pos, x_vel, x_ac, y_pos, y_vel, y_ac] = pos_vel[:, 0]

        mean = (x_pos, y_pos)

        # Plotting elipses
        plot_covariance(mean, cov=cov, fc='r', alpha=0.2)

        # plotting velocity
        plt.quiver(x_pos, y_pos, x_vel, y_vel, color='b', scale_units='xy', scale=1, alpha=0.8, width=0.0035)
        # plotting acceleration
        plt.quiver(x_pos, y_pos, x_ac, y_ac, color='y', scale_units='xy', scale=1, alpha=0.8, width=0.0035)

    x = []
    y = []
    for measure in measurements:
        if not measure: continue
        x.append(measure[0])
        y.append(measure[1])

    plt.plot(x, y, 'og', lw=1, ls='--')

    # plt.axis('equal')
    plt.show()


if __name__ == '__main__':

    a = KalmanTracker(0, 0)

    # lineal example
    # zs = ([0,0],[1,1],[2,2],[3,3],[4,4],[5,5],None,None,None,None,[10,10],None,None,[13,13],[14,14],[15,15]) #Andando en diagonal
    # Curve Example
    # zs = ([0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 4], [6, 4], [7, 4], None, None, None, [13,3],[14,2],[15,1],[16,0], None, None)

    zs = [] #Curva sinusoidal
    x = np.arange(0, 20, 0.5)
    for xn in x:
        zs.append([xn,np.sin(xn)])

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
    #

    last_time = 0  # tiempo en el que se tomo la ultima medida

    predictions = []
    covariances = []
    for index, z in enumerate(zs):

        if index == 0: continue  # la primera medicion la usamos para inizializar el tracker

        print (".........Medicion......... ", z)
        dt = 1
        last_time = index
        predX, predP = a.predict_with_time_diff(dt)
        [x_pos, x_vel, x_ac, y_pos, y_vel, y_ac] = predX[:, 0]

        predictions.append(predX)
        covariances.append(predP)

        print "---- Predict ----"
        print "Posicion", x_pos, y_pos
        print "Velocidad", x_vel, y_vel
        print "Aceleracion ", x_ac, y_ac
        print "Probabilidad", np.diag(predP)

        # Si no hay medicion no actualizo el filtro
        if not z:
            continue

        upX, upP = a.update(z)
        [x_pos, x_vel, x_ac, y_pos, y_vel, y_ac] = upX[:, 0]

        print "---- Update ----"
        print "Posicion", x_pos, y_pos
        print "Velocidad", x_vel, y_vel
        print "Aceleracion ", x_ac, y_ac
        print "Probabilidad", np.diag(upP)

        print ("----------------------------")

    plotResults(zs, predictions, covariances)

