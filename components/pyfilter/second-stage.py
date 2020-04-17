
import json, sys, pickle, time, random
import itertools
from PySide2 import QtWidgets, QtCore, QtGui
import numpy as np
import hdbscan
from pyclustering.cluster.dbscan import dbscan

from pyclustering.cluster.kmeans import kmeans, kmeans_visualizer
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer
from pyclustering.cluster.elbow import elbow
from pyclustering.cluster.gmeans import gmeans


FILE = 'tracklets.pickle'

class Escena(QtWidgets.QWidget):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.resize(QtWidgets.QDesktopWidget().availableGeometry(self).size() * 0.6);
        self.scene = QtWidgets.QGraphicsScene()
        self.scene.setSceneRect(-3000, -4000, 6000, 8000)
        self.view = QtWidgets.QGraphicsView(self.scene)
        self.view.resize(self.size())
        self.view.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
        self.view.setParent(self)
        self.view.setTransformationAnchor(QtWidgets.QGraphicsView.NoAnchor)
        self.view.setResizeAnchor(QtWidgets.QGraphicsView.NoAnchor)
        self.view.scale(3, -3)
        self.lines = []
        self.show()

    def wheelEvent(self, event):
        zoomInFactor = 1.25
        zoomOutFactor = 1 / zoomInFactor
        # Zoom
        if event.delta() > 0:
            zoomFactor = zoomInFactor
        else:
            zoomFactor = zoomOutFactor
        self.view.scale(zoomFactor, zoomFactor)

    def resizeEvent(self, event):
        self.view.resize(self.size())
    
    def draw(self, sample):
        colors = QtGui.QColor.colorNames()
        color = colors[random.randint(0, len(colors)-1)]
        [h,t,f,l,v,roi] = sample 
        self.scene.addLine(h[0],h[1],t[0],t[1], pen = QtGui.QPen(QtGui.QColor(color), 15))
        
        # for p in points:
        #     x = p['world'][0] 
        #     z = p['world'][2]
        #     self.scene.addEllipse(x-50,z-50,100,100, pen = QtGui.QPen(QtGui.QColor('orange'), 5))
        
    def drawTrack(self, cluster):
            colors = QtGui.QColor.colorNames()
            color = colors[random.randint(0, len(colors)-1)]
            # for line in self.lines:
            #     self.scene.removeItem(line)    
            #self.scene.clear()
            #for c in cluster:    
            #    self.scene.addLine(c[0][0], c[0][1], c[1][0], c[1][1], pen=QtGui.QPen(QtGui.QColor(color), 60))
            self.scene.addLine(cluster[0][0][0], cluster[0][0][1], cluster[-1][1][0], cluster[-1][1][1], pen=QtGui.QPen(QtGui.QColor(color), 60))

def dataIter(init=0, size=40, end=-1):
    with open(FILE, 'rb') as f:
        tracklets = pickle.load(f)

    print("Total evidences: ", len(tracklets))
    if end == -1:
        end = len(tracklets)
    while init+size < end:
        track = tracklets[init:init+size]
        init = init + size//2
        yield track

@QtCore.Slot()
def work():
    now = time.time()
    try:
        tracks = next(data_generator)
        #[h,t,f,l,v,s] = track[0]
       
        # for t in tracks:
        #     escena.draw(t)

        #X = np.array([t[5] for t in tracks])
        # X = np.array([(t[1]-t[0])/2 for t in tracks])

        # hdb = hdbscan.HDBSCAN(min_cluster_size=3)
        # hdb.fit(X)
        # print("Second stage HDBScan num clusters: ", len(set(hdb.labels_)))
        # # create a list og lists
        # clusters = [[] for i in range(len(set(hdb.labels_)))]
        # for i,l in enumerate(tracks):
        #     clusters[hdb.labels_[i]].append(l)

        # create instance of Elbow method using K value from 1 to 10.
        # kmin, kmax = 1, 5
        # #X = [(t[1]-t[0])/2 for t in tracks]
        # X = [t[5] for t in tracks]    
        # elbow_instance = elbow(X, kmin, kmax)
        # # process input data and obtain results of analysis
        # elbow_instance.process()
        # amount_clusters = elbow_instance.get_amount()  # most probable amount of clusters
        # wce = elbow_instance.get_wce()  # total within-cluster errors for each K
        # # perform cluster analysis using K-Means algorithm
        # centers = kmeans_plusplus_initializer(X, amount_clusters, amount_candidates=kmeans_plusplus_initializer.FARTHEST_CENTER_CANDIDATE).initialize()
        # kmeans_instance = kmeans(X, centers)
        # kmeans_instance.process()
        # # obtain clustering results and visualize them
        # clusters = kmeans_instance.get_clusters()
        # centers = kmeans_instance.get_centers()
        
        # Create DBSCAN algorithm.
        XD = [[] for i in range(len(tracks))]
        for i,t1 in enumerate(tracks):
            for j,t2 in enumerate(tracks):
                XD[i].append(np.linalg.norm(t1[0]-t2[1]) + np.linalg.norm(t1[1]-t2[0]))
        
        dbscan_instance = dbscan(XD, 700, 2, data_type='distance_matrix')
        # Start processing by DBSCAN.
        dbscan_instance.process()
        # Obtain results of clustering.
        clusters = dbscan_instance.get_clusters()
        noise = dbscan_instance.get_noise()

        # create a list og lists
        print(len(clusters), clusters)
        clouds = [[] for i in range(len(clusters))]
        for i,c in enumerate(clusters):
            for x in c:
                clouds[i].append(tracks[x])

        #kmeans_visualizer.show_clusters(X, clusters, centers)
        # print("Tracks ", len(set(hdb.labels_)))

        # escena.drawTrack(clusters)
        #print("real elapsed", (time.time() - now)*1000, " computed: " , s[-1]['timestamp']-s[0]['timestamp'])        
        
        for c in clouds:
            escena.drawTrack(c)
     #   escena.drawTrack(clusters)
    

    except StopIteration:
        print("End iterator")
        timer.stop()

###########################################
app = QtWidgets.QApplication(sys.argv)
timer = QtCore.QTimer()
escena = Escena()

INICIO = 0
SIZE = 10
FIN = -1

#data, sample, comb, n_comb, d_comb = readData(SIZE, OFFSET_FROM_END)
start = time.time()
#tracklets = deque(maxlen=50)
tracklets = []

data_generator = dataIter(INICIO, SIZE, FIN)
timer.timeout.connect(work)
#timer.setSingleShot(True)
timer.start(10)

sys.exit(app.exec_())
