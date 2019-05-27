import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from filterpy.stats import plot_covariance_ellipse
from scipy.linalg import block_diag
import matplotlib.pyplot as plt

#Asumo que la velocidad inicial es 0
class KalmanTracker:

    def __init__(self, x, y):
        self.tracker = KalmanFilter(dim_x=4, dim_z=2)
        dt = 1.0  # time step

        # State Transition Function
        self.tracker.F = np.array([[1, dt, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, dt],
                                   [0, 0, 0, 1]])

        # Measurement Function
        self.tracker.H = np.array([[1, 0, 0, 0],
                                   [0, 0, 1, 0]])

        # Measurement Noise Matrix
        self.tracker.R = np.eye(2)*0.05

        # Process Noise Matrix
        q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01)
        self.tracker.Q = block_diag(q, q)

        # Initial Position X, vel X, Y, vel Y
        self.tracker.x = np.array([[x, 0, y, 0]]).T

        #Covariance Matrix
        self.tracker.P = np.eye(4) * 5

        self.to_plotP = []
        self.to_plotx = []

    def predict(self,dt):

        self.tracker.x = self.tracker.x_post
        self.tracker.F = np.array([[1, dt, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, dt],
                                   [0, 0, 0, 1]])



        self.tracker.predict()

        self.to_plotP.append(self.tracker.P)
        self.to_plotx.append(self.tracker.x)

        return self.tracker.x, self.tracker.P


    def update(self,z):
        self.tracker.update(z)
        return self.tracker.x, self.tracker.P

    def plotResults(self,zs):


        for x, P in zip(self.to_plotx, self.to_plotP):

            cov = np.array([[P[0, 0], P[2, 0]],[P[0, 2], P[2, 2]]])
            mean = (x[0, 0], x[2, 0])
            # Plotting elipses
            plot_covariance_ellipse(mean, cov=cov, fc='r', alpha=0.20)

            # plotting velocity
            plt.quiver(x[0],x[2], x[1],x[3], color='b', scale_units='xy', scale=1,alpha=0.50,width=0.005)

        #Plotting meditions
        x = []
        y = []
        for z in zs:
            if not z: continue
            x.append(z[0])
            y.append(z[1])

        plt.plot(x, y, 'og', lw=1, ls='--')



        plt.axis('equal')
        plt.show()


zs = ([0,0],[1,1],[2,2],[3,3],[4,4],[5,5],None,None,None,None,[10,10],None,None,[13,13],[14,14],[15,15]) #Andando en diagonal

person_tracker = KalmanTracker(0,0)
last_time = 0 #tiempo en el que se tomo la ultima medida

for index, z in enumerate(zs):

    if index == 0: continue #la primera medicion la usamos para inizializar el tracker

    print (".........Medicion......... ", z)
    dt = index - last_time
    predX, predP = person_tracker.predict(dt)

    print "Predicha", predX[0], predX[2], "Con velocidad", predX[1], predX[3]

    # Si no hay medicion no actualizo el filtro
    if not z:
        continue

    upX,upP = person_tracker.update(z)
    last_time = index
    print "Adaptada", upX[0],upX[2], "Con velocidad", upX[1],upX[3]
    print "Siguiente probabilidad", np.diag(upP)

    print ("----------------------------")


person_tracker.plotResults(zs)
