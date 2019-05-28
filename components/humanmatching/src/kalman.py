import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from filterpy.stats import plot_covariance_ellipse, plot_covariance
from scipy.linalg import block_diag
# import matplotlib.pyplot as plt



class KalmanTracker(KalmanFilter):
    def __init__(self, x=0, y=0):
        super(KalmanTracker, self).__init__(dim_x=4, dim_z=2)
        # Measurement Function
        self.H = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])

        # Measurement Noise Matrix
        self.R = np.eye(2) * 0.1  # 0.1 meters error
        # Process Noise Matrix
        q = Q_discrete_white_noise(dim=2, dt=1, var=0.01)
        self.Q = block_diag(q, q)

        # Initial Position X, vel X, Y, vel Y
        self.x = np.array([[x, 0, y, 0]]).T

        # Covariance Matrix
        self.P = np.eye(4) * 2.5


    def predict_with_time_diff(self,dt):

        self.x = self.x_post
        self.F = np.array([[1, dt, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, dt],
                           [0, 0, 0, 1]])

        # # Process Noise Matrix
        # q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01)
        # self.Q = block_diag(q, q)

        self.predict()

        return self.x, self.P

    def update(self,z):
        super(KalmanTracker, self).update(z)
        return self.x, self.P


# def plotResults(measurements, predictions, covariances):
#
#         for pos_vel, cov_matrix in zip(predictions, covariances):
#
#             cov = np.array([[cov_matrix[0, 0], cov_matrix[2, 0]],[cov_matrix[0, 2], cov_matrix[2, 2]]])
#             [x_pos, x_vel, y_pos, y_vel] = pos_vel[:,0]
#             mean = (x_pos, y_pos)
#             # Plotting elipses
#             plot_covariance_ellipse(mean, cov=cov, fc='r', alpha=0.20)
#
#             # plotting velocity
#             plt.quiver(x_pos,y_pos, x_vel,y_vel, color='b', scale_units='xy', scale=1,alpha=0.50,width=0.005)
#
#         #Plotting meditions
#         x = []
#         y = []
#         for measure in measurements:
#             if not measure: continue
#             x.append(measure[0])
#             y.append(measure[1])
#
#         plt.plot(x, y, 'og', lw=1, ls='--')
#
#
#
#         plt.axis('equal')
#         plt.show()


if __name__ == '__main__':

    a = KalmanTracker(0, 0)

    # lineal example
    # zs = ([0,0],[1,1],[2,2],[3,3],[4,4],[5,5],None,None,None,None,[10,10],None,None,[13,13],[14,14],[15,15]) #Andando en diagonal
    # Curve Example
    zs = ([0,0],[1,1],[2,2],[3,3],[4,4],[5,4], [6,4], [7,4], None, None, [10,4], [11,3], [12,2], None, None)

    last_time = 0 #tiempo en el que se tomo la ultima medida

    predictions = []
    covariances = []
    for index, z in enumerate(zs):

        if index == 0: continue #la primera medicion la usamos para inizializar el tracker

        print (".........Medicion......... ", z)
        # dt = 1
        # last_time = index
        dt = index - last_time
        predX, predP = a.predict_with_time_diff(dt)
        # predX, predP = person_tracker.predict(dt)
        predictions.append(predX)
        covariances.append(predP)


        print "Predicha", predX[0], predX[2], "Con velocidad", predX[1], predX[3]

        # Si no hay medicion no actualizo el filtro
        if not z:
            continue

        upX,upP = a.update(z)
        # upX,upP = person_tracker.update(z)
        last_time = index
        print "Adaptada", upX[0],upX[2], "Con velocidad", upX[1],upX[3]
        print "Siguiente probabilidad", np.diag(upP)

        print ("----------------------------")

    plotResults(zs, predictions, covariances)
