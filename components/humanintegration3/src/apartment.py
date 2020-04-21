import random, json
from PySide2.QtWidgets import QGraphicsScene, QDesktopWidget, QGraphicsView, QGraphicsPixmapItem
from PySide2.QtCore import Qt, QRectF, QPointF
from PySide2.QtGui import QPen, QColor, QBrush, QPixmap

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

        self.persons = {}
        self.pixmapSize = (0, 0)
        self.initializeWorld()
        
    def addPerson(self, pos, angle=0, color=-1, size=100):
        colors = QColor.colorNames()
        color = colors[random.randint(0, len(colors)-1)] if color==-1 else color
        pos = [pos[0],pos[2]] if len(pos)>2 else pos
        p = self.scene.addEllipse(pos[0]-size//2, pos[1]-size//2, size, size, pen=QPen(QColor(color),20), brush=QBrush(color=QColor(color)))

        # pixmap
        pixmap = QPixmap("person.png").scaled(600, 300)
        self.pixmapSize = (pixmap.width() / 2, pixmap.height() / 2)
        pixItem = QGraphicsPixmapItem(pixmap)
        pixItem.setTransformOriginPoint(pixItem.boundingRect().center())
        pixItem.setZValue(20)
        self.scene.addItem(pixItem)

        self.persons[p] = pixItem

        return p

    def movePerson(self, elipse, pos, size=100):
        #elipse.setPos(pos[0], pos[1])
        pos = [pos[0],pos[2]] if len(pos)>2 else pos
        color=elipse.pen().color()
        self.scene.addEllipse(pos[0]-size//2, pos[1]-size//2, size, size, pen=QPen(QColor(color),20), brush=QBrush(color=QColor(color)))
        # pixmap
        self.persons[elipse].setPos(pos[0]-self.pixmapSize[0], pos[1]-self.pixmapSize[1])

        # change rotation value when provided
        self.persons[elipse].setRotation(180)


    def wheelEvent(self, event):
        zoomInFactor = 1.15
        zoomOutFactor = 1 / zoomInFactor
        # Zoom
        if event.delta() > 0:
            zoomFactor = zoomInFactor
        else:
            zoomFactor = zoomOutFactor
        self.graphicsView.scale(zoomFactor, zoomFactor)
        
    def initializeWorld(self):
        with open('autonomy.json','r') as f:
            world = json.load(f)
       
        #load dimensions
        dim = world["dimensions"]
        x_offset = -3200
        y_offset = 1850

        # load roundtables
        # for k,v in world["roundTables"].items():
        #     print(v)
        #     box = self.scene.addEllipse(QRectF(-v[2]// 2, -v[3]// 2, v[2], v[3]), QPen(QColor("Khaki")), QBrush(QColor("Khaki")));
        #     box.setPos(v[4]+x_offset, v[5]+x_offset);
        #     self.boxes.append(box)
        
        # load tables
        for k,v in world["tables"].items():
            box = self.scene.addRect(QRectF(-v[2]// 2, -v[3]//2, v[2], v[3]), QPen(QColor("SandyBrown")), QBrush(QColor("SandyBrown")))
            box.setPos(v[4]+x_offset, v[5]+y_offset)
            box.setTransformOriginPoint(box.mapFromScene(QPointF(0,0)))
            box.setRotation(v[6])
            self.boxes.append(box)
        
        # load walls
        for k,v in world['walls'].items():
            box = self.scene.addRect(QRectF(-v[2]// 2, -v[3]//2, v[2], v[3]), QPen(QColor("Brown")), QBrush(QColor("Brown")))
            box.setPos(v[4]+x_offset, v[5]+y_offset);
            box.setTransformOriginPoint(box.mapFromScene(QPointF(0,0)))
            box.setRotation(v[6]);
            self.boxes.append(box);
        # }

        # //load points
        # QVariantMap points = mainMap[QString("points")].toMap();
        # for (auto &t : points)
        # {
        #     QVariantList object = t.toList();
        #     auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
        #     box->setPos(object[4].toFloat()+x_offset, object[5].toFloat()+y_offset);
        #     boxes.push_back(box);
        # }
        # //load boxes
        # QVariantMap cajas = mainMap[QString("boxes")].toMap();
        # for (auto &t : cajas)
        # {
        #     QVariantList object = t.toList();
        #     auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Orange")));
        #     box->setPos(object[4].toFloat()+x_offset, object[5].toFloat()+y_offset);
        #     //box->setPos(object[4].toFloat(), object[5].toFloat());
        #     //box->setRotation(object[6].toFloat()*180/M_PI2);
        #     box->setFlag(QGraphicsItem::ItemIsMovable);
        #     boxes.push_back(box);
        # }
        # QTransform t;
        # //t.translate(3200, -1850);
        # t.rotate(-90);
        # //t.translate(-3200, 1850);
        # for(auto &item : boxes)
        # {	
        #     item->setPos(t.map(item->pos()));
        #     item->setRotation(item->rotation() + 90);
        #     item->setPos(item->pos() + QPointF(1850,3200));
        # }
        # /////////////
        # //AXIS
        self.scene.addLine(0,0,400,0,QPen(QBrush(QColor("red")),20));
        self.scene.addLine(0,0,0,400,QPen(QBrush(QColor("blue")),20));




