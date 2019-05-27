# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/mainUI.ui',
# licensing of 'src/mainUI.ui' applies.
#
# Created: Tue May 21 20:01:28 2019
#      by: pyside2-uic  running on PySide2 5.12.1
#
# WARNING! All changes made in this file will be lost!

from PySide2 import QtCore, QtGui, QtWidgets

class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        guiDlg.setObjectName("guiDlg")
        guiDlg.resize(731, 572)
        self.verticalLayout = QtWidgets.QVBoxLayout(guiDlg)
        self.verticalLayout.setObjectName("verticalLayout")
        self._graph_view = QNetworkxWidget(guiDlg)
        self._graph_view.setObjectName("_graph_view")
        self.verticalLayout.addWidget(self._graph_view)
        self._maps_layout = QtWidgets.QHBoxLayout()
        self._maps_layout.setObjectName("_maps_layout")
        self._first_view = HumanVisualizationWidget(guiDlg)
        self._first_view.setObjectName("_first_view")
        self._maps_layout.addWidget(self._first_view)
        self._second_view = HumanVisualizationWidget(guiDlg)
        self._second_view.setObjectName("_second_view")
        self._maps_layout.addWidget(self._second_view)
        self.verticalLayout.addLayout(self._maps_layout)
        self._noise_layout = QtWidgets.QHBoxLayout()
        self._noise_layout.setObjectName("_noise_layout")
        self._noise_checkbox = QtWidgets.QCheckBox(guiDlg)
        self._noise_checkbox.setObjectName("_noise_checkbox")
        self._noise_layout.addWidget(self._noise_checkbox)
        self._noise_slider = QtWidgets.QSlider(guiDlg)
        self._noise_slider.setMaximum(100)
        self._noise_slider.setPageStep(100)
        self._noise_slider.setOrientation(QtCore.Qt.Horizontal)
        self._noise_slider.setObjectName("_noise_slider")
        self._noise_layout.addWidget(self._noise_slider)
        self.verticalLayout.addLayout(self._noise_layout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self._noise_factor_lcd_layout = QtWidgets.QHBoxLayout()
        self._noise_factor_lcd_layout.setObjectName("_noise_factor_lcd_layout")
        self._noise_factor_label = QtWidgets.QLabel(guiDlg)
        self._noise_factor_label.setObjectName("_noise_factor_label")
        self._noise_factor_lcd_layout.addWidget(self._noise_factor_label)
        self._noise_factor_lcd = QtWidgets.QSpinBox(guiDlg)
        self._noise_factor_lcd.setReadOnly(True)
        self._noise_factor_lcd.setMaximum(10000)
        self._noise_factor_lcd.setObjectName("_noise_factor_lcd")
        self._noise_factor_lcd_layout.addWidget(self._noise_factor_lcd)
        self.horizontalLayout.addLayout(self._noise_factor_lcd_layout)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self._min_max_noise_layout = QtWidgets.QHBoxLayout()
        self._min_max_noise_layout.setObjectName("_min_max_noise_layout")
        self._min_max_label = QtWidgets.QLabel(guiDlg)
        self._min_max_label.setObjectName("_min_max_label")
        self._min_max_noise_layout.addWidget(self._min_max_label)
        self._min_noise = QtWidgets.QSpinBox(guiDlg)
        self._min_noise.setMinimum(-5000)
        self._min_noise.setMaximum(5000)
        self._min_noise.setObjectName("_min_noise")
        self._min_max_noise_layout.addWidget(self._min_noise)
        self._max_noise = QtWidgets.QSpinBox(guiDlg)
        self._max_noise.setMinimum(-5000)
        self._max_noise.setMaximum(5000)
        self._max_noise.setObjectName("_max_noise")
        self._min_max_noise_layout.addWidget(self._max_noise)
        self.horizontalLayout.addLayout(self._min_max_noise_layout)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.verticalLayout.setStretch(0, 6)
        self.verticalLayout.setStretch(1, 5)

        self.retranslateUi(guiDlg)
        QtCore.QMetaObject.connectSlotsByName(guiDlg)

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QtWidgets.QApplication.translate("guiDlg", "humanmatching", None, -1))
        self._noise_checkbox.setText(QtWidgets.QApplication.translate("guiDlg", "Apply Noise", None, -1))
        self._noise_factor_label.setText(QtWidgets.QApplication.translate("guiDlg", "Noise Factor:", None, -1))
        self._min_max_label.setText(QtWidgets.QApplication.translate("guiDlg", "Min, max noise added:", None, -1))
        self._min_noise.setSuffix(QtWidgets.QApplication.translate("guiDlg", "mm", None, -1))
        self._max_noise.setSuffix(QtWidgets.QApplication.translate("guiDlg", "mm", None, -1))

from libs.QNetworkxGraph.QNetworkxGraph import QNetworkxWidget
from libs.HumanVisualizationWidget import HumanVisualizationWidget
