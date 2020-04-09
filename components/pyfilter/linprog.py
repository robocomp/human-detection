import json, sys, pickle
import random
import numpy as np
import pprint
import itertools
import vg
import gurobipy as gp
from gurobipy import GRB
from PySide2.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QWidget, QDesktopWidget
from PySide2.QtCore import QSizeF, QPointF
from PySide2.QtGui import QPolygonF, QPen, QColor, QBrush
from PySide2 import QtCore, QtGui
import cv2

FILE = 'human_data_2p_descriptores.txt'
FILE = 'human_data_3C_2P_I_L3.txt'
    
def readData(size=30, offset=50):
    pp = pprint.PrettyPrinter(indent=4)
    with open(FILE, 'r') as f:
        raw = f.read()

    data = json.loads(raw)

    #pp.pprint(data["data_set"][0]["world"])
    total = len(data["data_set"])
    print("Total evidences: ", total)
    # pick a subset
    sample = data["data_set"][total-offset:total-offset+size]
    addVelocity(sample)
    print("Sample elapsed time (ms): ", sample[-1]['timestamp']-sample[0]['timestamp'])
    print("Sample size:", len(sample))
    data = removeDuplicates(data)    
    # compute combination of N samples taken as 2
    n_comb = list(itertools.combinations(range(len(sample)), 2))
    comb = list(itertools.combinations(sample, 2))
    d_comb = dict()
    for i,c in enumerate(n_comb):
        d_comb[c] = i
    #comb = list(itertools.permutations(sample, 2))
    print("Number of combinations:", len(comb))
    return data['data_set'], sample, comb, n_comb, d_comb

def removeDuplicates(sample):
    result = []
    toRemove = []
    for i,s in enumerate(sample):
            if s['world'] not in result:
                result.append(s['world'])
            else:
                toRemove.append(i)
    for i in reversed(toRemove):
        del sample[i]
    if len(sample) != len(result):
        print("Duplicates removed: ", len(sample)-len(result))
    return sample

def dataIter(init=0, size=40, end=-1):
    #pp = pprint.PrettyPrinter(indent=4)
    with open(FILE, 'r') as f:
        raw = f.read()
    data = json.loads(raw)["data_set"]
    #data = removeDuplicates(data)
    #pp.pprint(data["data_set"][0]["world"])
    total = len(data)
    # remove duplicates
    print("Total evidences: ", total)
    # loop
    if end == -1:
        end = len(data)
    while init+size < end:
        init = init + size//2
        sample = data[init:init+size]
        addVelocity(sample)
        print("Sample elapsed time (ms): ", sample[-1]['timestamp']-sample[0]['timestamp'])
        print("Sample size:", len(sample))
         # compute combination of N samples taken as 2
        comb = list(itertools.combinations(sample, 2))      # could be permutations
        n_comb = list(itertools.combinations(range(len(sample)), 2))
        d_comb = dict()  # d_comb = { c:i for i,c in enumerate(n_comb) }
        for i,c in enumerate(n_comb):
            d_comb[c] = i
        print("Number of combinations:", len(comb))
        yield sample, comb, n_comb, d_comb


def spaceTimeAffinitiy(d1, d2):
    # max[1 - B(e(D1,D2) + e(D2,D1)), 0]
    # e(D1,D2) = ||q_1 - p_2|| , q_1 = p_1 + v_1(t2-t1) of D1
    # return stA
    BETA = 1.0
    dist = np.linalg.norm
    q1 = np.array(d1['world'][0:3]) + np.array(d1['l_velocity']) * np.abs(d2['timestamp']-d1['timestamp'])
    e12 = dist(q1 - np.array(d2['world'][0:3]))
    q2 = np.array(d2['world'][0:3]) + np.array(d2['l_velocity']) * np.abs(d2['timestamp']-d1['timestamp'])
    e21 = dist(q2 - np.array(d1['world'][0:3]))
    #print(e12,)
    return max(1 - BETA*(e12 + e21)/1000, 0)
    
def appearanceAffinity(d1, d2):
    # max[1 - a d(f_1, f_2), 0]
    # d(.) can be the scalar product of descriptor vectors
    ALFA = 1
    flatten = itertools.chain.from_iterable
    d1_desc =[k for [k,v] in d1['joints'].items()]
    d2_desc =[k for [k,v] in d2['joints'].items()]
    # compute the set of common keys
    common = list(set(d1_desc).intersection(set(d2_desc)))
    # # 6 is the position of desc values
    a = vg.normalize(np.array(list(flatten([v[6] for [k,v] in d1['joints'].items() if k in common]))))
    b = vg.normalize(np.array(list(flatten([v[6] for [k,v] in d2['joints'].items() if k in common]))))
    try:
        r = np.einsum('ij, ij->i', a, b)
        return np.median(r)
    except:
        #print("wrong eisum")
        return 0
    
def totalAffinity(spatial, aspect):
    # if aA * stA = 0: -inf
    # if sA * stA = 1: inf
    # tA = -1 + 2/(1+exp(-l(aA*stA-mu))))
    LANDA = 1.0
    MU = 0.25
    p = aspect*spatial
    if p == 0:
        return -1000
    if p == 1:
        return 1000
    return -1.0 + 2.0/(1.0 + np.exp(-LANDA*(p-MU)))
    
def addVelocity(sample):
    # sort sample by timestamp
    s_sample = sorted(sample, key=lambda x: x['timestamp'])
    # compute the feature vectors for each observation  [aspect, position, time, velocity]

    # to compute the velocity we search in the time neighborhood of each observation, the closest observation in space.
    # if the time difference times the maximum allowed speed is less that that distance, it is added to the pool
    # final velocity is the component-wise median of the pool
    MAX_TIME_THRESHOLD = 1000 # ms 
    MAX_VEL_ALLOWED = 600 # mm/sg
    norm = np.linalg.norm
    for i in range(len(s_sample)):
        cur = s_sample[i]
        cur_pos = np.array(cur['world'][0:3])
        cur_time = cur['timestamp']
        vels = [(cur_pos-np.array(x['world'][0:3])) / (cur_time-x['timestamp']) for x in s_sample if 0 < np.abs(cur_time - x['timestamp']) <= MAX_TIME_THRESHOLD]
        valid_vels = [x for x in vels if norm(x) < MAX_VEL_ALLOWED]
        s_sample[i]['l_velocity'] = np.median(valid_vels, axis=0).tolist()

    return s_sample        

def correlations(comb):
    l_corr = list()
    for d1,d2 in comb:
        sp = spaceTimeAffinitiy(d1,d2)
        sa =appearanceAffinity(d1,d2)
        l_corr.append(totalAffinity(sp, sa))
    return np.array(l_corr)

def connected_components(neighbors):
    seen = set()
    def component(node):
        nodes = set([node])
        while nodes:
            node = nodes.pop()
            seen.add(node)
            nodes |= neighbors[node] - seen
            yield node
    for node in neighbors:
        if node not in seen:
            yield component(node)

def optimize(corr, comb, n_comb, d_comb):
    try:
        x = m.addMVar(shape=len(comb), vtype= GRB.BINARY, name="x")

        # # set goal
        m.setObjective( corr @ x, GRB.MAXIMIZE)

        # # add constraints
        # # x_u_v + x_v_t - x_u_t <= 1

        for n in n_comb:
            f,s = n
            for i in range(len(comb)):
                if (s,i) in n_comb and (f,i) in n_comb:
                    # print("(", n, ") + (", s, i, ") - (", f, i, ")" )
                    # print(d_comb[n], d_comb[s,i], d_comb[f,i])
                    m.addConstr(x[d_comb[f,s]] + x[d_comb[s,i]] - x[d_comb[f,i]] <= 1)

        m.optimize()

        # convert results to list  
        print('Obj: %g' % m.objVal)   
        return list(m.getVars())

    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ': ' + str(e))
    except AttributeError:
        print('Encountered an attribute error')

def partitionGraph(l_vars, n_comb, d_comb, sample):            
    # construct a graph and compute connected components
    graph = {node: set(s for f,s in n_comb if f==node and l_vars[d_comb[f,s]].x == 1) 
                                                    for node in range(len(sample)) }
    components = list()
    for component in connected_components(graph):
        components.append(list(component))
        
    return components
###########################################

class Escena(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.resize(QDesktopWidget().availableGeometry(self).size() * 0.6);
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(-3000, -4000, 6000, 8000)
        self.view = QGraphicsView(self.scene)
        self.view.scale(1, -1)
        self.view.resize(self.size())
        self.view.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
        self.view.setParent(self)
        self.view.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.view.setResizeAnchor(QGraphicsView.NoAnchor)
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
    
    def draw(self, comps, sample):
        colors = ["red","blue","green","brown","cyan","black","darkRed","darkBlue","darkCyan"]
        colors = QColor.colorNames()

        for co in sample:
            pol = QPolygonF()
            x, _, z, _ = co['world'] 
            pol.append(QPointF(x,z))
            self.scene.addEllipse(x,z,50,50, pen=QPen(QColor('orange'), 100), brush=QBrush(QColor('orange')))
        
        x,_,z,_ = sample[-1]['world']
        self.scene.addRect(x,z,60,60, pen=QPen(QColor('red'), 100))        
        x,_,z,_ = sample[0]['world']
        self.scene.addRect(x,z,60,60, pen=QPen(QColor('green'), 100))        

        for co in comps:
            if len(co)>4:
                color = colors[random.randint(0, len(comps))]
                for c in co:   
                    x, _, z, _ = sample[c]['world'] 
                    self.scene.addEllipse(x,z,30,30, pen=QPen(QColor(color), 10), brush=QBrush(style=QtCore.Qt.SolidPattern, color=QColor(color)))

###########################################
app = QApplication(sys.argv)
escena = Escena()

OFFSET_FROM_END=2000
SIZE=30

#data, sample, comb, n_comb, d_comb = readData(SIZE, OFFSET_FROM_END)
m = gp.Model("people")
data_generator = dataIter(0, 40, -1)

# corr = correlations(comb)
# l_vars = optimize(corr, comb, n_comb, d_comb)
# components = partitionGraph(l_vars, n_comb, d_comb, sample)

for sample, comb, n_comb, d_comb in data_generator:
    corr = correlations(comb)
    l_vars = optimize(corr, comb, n_comb, d_comb)
    components =partitionGraph(l_vars, n_comb, d_comb, sample)
    escena.draw(components, sample)
    app.processEvents()
    print(components)

# with open(FILE + '_' + str(SIZE) + '.pa',mode = 'rb') as f:
#     components = pickle.load(f)

# with open(FILE + '_' + str(SIZE) + '.pa',mode = 'wb') as f:
#     pickle.dump(components, f)
#print(components)

#escena.draw(components, sample)
sys.exit(app.exec_())

