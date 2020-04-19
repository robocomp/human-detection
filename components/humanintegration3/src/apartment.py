import random
from PySide2.QtWidgets import QGraphicsScene, QDesktopWidget, QGraphicsView
from PySide2.QtCore import Qt, QRectF
from PySide2.QtGui import QPen, QColor, QBrush

class Apartment2D(object):
    def __init__(self, ui):
        #self.ui = ui
        #self.ui.guiDlg.resize(QDesktopWidget().availableGeometry(self).size() * 0.6)
        self.scene = QGraphicsScene()
        self.dim = {"HMIN":-3000, "VMIN":-4000, "WIDTH":6000,"HEIGHT":8000}
        self.scene.setSceneRect(self.dim['HMIN'], self.dim['VMIN'], self.dim['WIDTH'], self.dim['HEIGHT'])
        ui.graphicsView.setScene(self.scene)
        self.boxes = []
        #self.ui.graphicsView.setViewport(QGLWidget())
        ui.graphicsView.scale(1,-1)
        ui.graphicsView.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        ui.graphicsView.setTransformationAnchor(QGraphicsView.NoAnchor)
        ui.graphicsView.setResizeAnchor(QGraphicsView.NoAnchor)
        
    def addPerson(self, pos, angle=0, color=-1, size=200, leave_trail=False):
        colors = QColor.colorNames()
        color = colors[random.randint(0, len(colors)-1)] if color==-1 else color
        pos = [pos[0],pos[2]] if len(pos)>2 else pos
        p = self.scene.addEllipse(pos[0]-size//2, pos[1]-size//2, size, size, pen=QPen(QColor(color),50), brush=QBrush(color=QColor(color)))
        return p

    def movePerson(self, elipse, pos, size=200):
        #elipse.setPos(pos[0], pos[1])
        pos = [pos[0],pos[2]] if len(pos)>2 else pos
        color=elipse.pen().color()
        self.scene.addEllipse(pos[0]-size//2, pos[1]-size//2, size, size, pen=QPen(QColor(color),50), brush=QBrush(color=QColor(color)))
            

    def wheelEvent(self, event):
        zoomInFactor = 1.15
        zoomOutFactor = 1 / zoomInFactor
        # Zoom
        if event.delta() > 0:
            zoomFactor = zoomInFactor
        else:
            zoomFactor = zoomOutFactor
        self.graphicsView.scale(zoomFactor, zoomFactor)
        





