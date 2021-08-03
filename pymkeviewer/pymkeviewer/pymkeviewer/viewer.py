#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    Python pointcloud viewer PySide2 GUI definitions
"""
__author__ = 'Sujay Jayashankar', 'Kiran NV', 'Karthik Raja S'
__copyright__ = 'Copyright (c) 2021, Magik-Eye Inc.'

# =============================================================================

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from .__init__ import __version__

# =============================================================================
# =============================================================================

color_options = ['autumn', 'blues', 'cool', 'greens', 'reds', 'spring',
                 'summer', 'fire', 'grays', 'hot', 'ice', 'RdYeBuCy',
                 'winter', 'light_blues', 'orange', 'viridis', 'coolwarm',
                 'PuGr', 'GrBu', 'GrBu_d', 'RdBu', 'cubehelix', 'single_hue',
                 'hsl', 'husl', 'diverging']

# =============================================================================
# =============================================================================


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.setStyleSheet(
            u"color: rgb(255, 255, 255); background-color: rgb(46, 52, 54);")

        # Menubar elements initialisation  :: Start
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 785, 22))
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuView = QMenu(self.menubar)
        self.menuView.setObjectName(u"menuView")
        self.menuHelp = QMenu(self.menubar)
        self.menuHelp.setObjectName(u"menuHelp")
        MainWindow.setMenuBar(self.menubar)
        # Menubar elements initialisation  :: End

        # Menubar Action elements intitialisation :: Start
        self.actionExit = QAction(MainWindow)
        self.actionExit.setObjectName(u"actionExit")
        self.actionAbout = QAction(MainWindow)
        self.actionAbout.setObjectName(u"actionAbout")
        self.actionAbout.triggered.connect(about_application)
        self.actionToolbox = QAction(MainWindow)
        self.actionToolbox.setObjectName(u"actionToolbox")
        self.actionToolbox.setCheckable(True)
        # self.actionToolbox.setChecked(True)
        self.actionToolbox.triggered.connect(self.toggletoolbar)
        # Menubar Action elements initialisation :: End

        # Main Application Graphical Layout :: Start
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.openGLWidget = QOpenGLWidget(self.centralwidget)
        self.openGLWidget.setObjectName(u"openGLWidget")

        self.openGLLoadingLabel = QLabel("Loading...")
        self.openGLLoadingLabel.setStyleSheet("color: white;")  # background-color: black; would need fixing margins...
        self.openGLLoadingLabel.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)

        self.centralStack = QStackedWidget(self.centralwidget)
        self.centralStack.setObjectName(u"centralStack")
        self.centralStack.addWidget(self.openGLLoadingLabel)
        self.centralStack.addWidget(self.openGLWidget)

        self.verticalLayout.addWidget(self.centralStack)

        MainWindow.setCentralWidget(self.centralwidget)
        # Main Application Graphical Layout :: End

        # Statusbar initialisation  :: Start
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
        # Statusbar initialisation  :: End

        # Dock Window and Dock Window elements initialisation :: Start
        self.dockWidget_2 = QDockWidget(MainWindow)
        self.dockWidget_2.setObjectName(u"dockWidget_2")
        self.dockWidget_2.setMinimumSize(QSize(250, 555))
        self.dockWidget_2.setAllowedAreas(
            Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.dockWidgetContents_2 = Update_dockwidget_dynamic()
        self.dockWidgetContents_2.setObjectName(u"dockWidgetContents_2")
        self.label_4 = QLabel(self.dockWidgetContents_2)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(10, 10, 200, 20))
        self.verticalLayoutWidget = QWidget(self.dockWidgetContents_2)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(10, 40, 200, 170))
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.groupBox = QGroupBox(self.verticalLayoutWidget)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(0, 0, 210, 170))
        self.groupBox.setStyleSheet(
            u"color: rgb(255, 255, 255); background-color: rgb(46, 52, 54);")
        self.formLayout = QFormLayout(self.groupBox)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setGeometry(QRect(0, 0, 210, 170))
        self.scrollArea = QScrollArea(self.dockWidgetContents_2)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 200, 170))
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout_2.addWidget(self.scrollArea)
        self.pushButtonRefresh = QPushButton(self.dockWidgetContents_2)
        self.pushButtonRefresh.setObjectName(u"pushButton")
        self.pushButtonRefresh.setGeometry(QRect(115, 220, 100, 25))
        self.pushButtonRefresh.setStyleSheet(
            u"color: rgb(0, 0, 0); background-color: rgb(186, 189, 182);")
        self.pushButtonConnect = QPushButton(self.dockWidgetContents_2)
        self.pushButtonConnect.setObjectName(u"pushButton")
        self.pushButtonConnect.setGeometry(QRect(10, 220, 100, 25))
        self.pushButtonConnect.setStyleSheet(
            u"color: rgb(0, 0, 0); background-color: rgb(186, 189, 182);")
        self.listView = QListView(self.dockWidgetContents_2)
        self.listView.setObjectName(u"listView")
        self.listView.setGeometry(QRect(10, 290, 202, 210))
        self.listView.setStyleSheet(u"color: rgb(255, 255, 255);")
        self.horizontalSlider = QSlider(self.dockWidgetContents_2)
        self.horizontalSlider.setObjectName(u"horizontalSlider")
        self.horizontalSlider.setGeometry(QRect(30, 420, 111, 20))
        self.horizontalSlider.setOrientation(Qt.Horizontal)
        self.horizontalSlider.setRange(0, 100)
        self.horizontalSlider.setValue(5)
        self.horizontalSlider.valueChanged.connect(self.sliderToolTip)
        self.spinBox = QSpinBox(self.dockWidgetContents_2)
        self.spinBox.setObjectName(u"spinBox")
        self.spinBox.setGeometry(QRect(160, 420, 48, 21))
        self.spinBox.setRange(0, 100)
        self.spinBox.setValue(5)
        self.spinBox.valueChanged.connect(self.sliderToolTip)
        self.label = QLabel(self.dockWidgetContents_2)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(30, 390, 200, 20))
        self.label_2 = QLabel(self.dockWidgetContents_2)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(30, 460, 91, 17))
        self.comboBox = QComboBox(self.dockWidgetContents_2)
        for i in range(0, 26):
            self.comboBox.addItem("")
        self.comboBox.setObjectName(u"comboBox")
        self.comboBox.setGeometry(QRect(120, 460, 86, 25))
        self.label_3 = QLabel(self.dockWidgetContents_2)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 260, 200, 20))
        self.pushButtonStart = QPushButton(self.dockWidgetContents_2)
        self.pushButtonStart.setObjectName(u"pushButtonStart")
        self.pushButtonStart.setGeometry(QRect(30, 310, 171, 25))
        self.checkBoxAxis = QCheckBox(self.dockWidgetContents_2)
        self.checkBoxAxis.setObjectName(u"checkBoxAxis")
        self.checkBoxAxis.setGeometry(QRect(30, 350, 171, 25))
        self.dockWidget_2.setWidget(self.dockWidgetContents_2)
        MainWindow.addDockWidget(Qt.RightDockWidgetArea, self.dockWidget_2)
        # Dock Window and Dock window elements initialisation :: End

        # Menubar action elements :: Start
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuView.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.menuFile.addAction(self.actionExit)
        self.menuView.addAction(self.actionToolbox)
        self.menuHelp.addAction(self.actionAbout)
        # Menubar action elements :: End

        # Status Bar elements initialisation :: Start
        self.spacer = QLabel()
        self.statusbar.addWidget(self.spacer, 0)
        self.fps = QLabel()
        self.fps.setObjectName(u"Render FPS: None")
        self.statusbar.addWidget(self.fps, 1)

        self.missed_frames = QLabel()
        self.missed_frames.setObjectName(u"Frame DPS:")
        self.statusbar.addWidget(self.missed_frames, 1)

        self.sensor_fps = QLabel()
        self.sensor_fps.setObjectName(u"Sensor FPS: None")
        self.statusbar.addWidget(self.sensor_fps, 1)
        # Status Bar elements initialisation :: End

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    
    # =========================================================================

    # setupUi
    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate(
            "MainWindow", u"PyMkEViewer", None))
        self.actionExit.setText(QCoreApplication.translate(
            "MainWindow", u"Exit", None))
        self.actionAbout.setText(QCoreApplication.translate(
            "MainWindow", u"About", None))
        self.actionToolbox.setText(QCoreApplication.translate(
            "MainWindow", u"Toolbox", None))
        self.menuFile.setTitle(QCoreApplication.translate(
            "MainWindow", u"File", None))
        self.menuView.setTitle(QCoreApplication.translate(
            "MainWindow", u"View", None))
        self.menuHelp.setTitle(QCoreApplication.translate(
            "MainWindow", u"Help", None))
        self.pushButtonRefresh.setText(QCoreApplication.translate(
            "MainWindow", u"Refresh", None))
        self.pushButtonConnect.setText(QCoreApplication.translate(
            "MainWindow", u"Connect", None))
        self.label.setText(QCoreApplication.translate(
            "MainWindow", u"Point Size", None))
        self.label_2.setText(QCoreApplication.translate(
            "MainWindow", u"Point Color", None))
        for i in range(0, 26):
            self.comboBox.setItemText(
                i, QCoreApplication.translate(
                    "MainWindow", color_options[i],
                    None))
        self.label_3.setText(QCoreApplication.translate(
            "MainWindow", u"Renderer   ", None))
        self.label_4.setText(
            QCoreApplication.translate(
                "MainWindow", u"Active Device List   ", None))
        self.pushButtonStart.setText(
            QCoreApplication.translate(
                "MainWindow", u"Start Rendering", None))
        self.checkBoxAxis.setText(
            QCoreApplication.translate(
                "MainWindow", u"Show Axes", None))
        self.checkBoxAxis.setChecked(True)
        self.fps.setText(QCoreApplication.translate(
            "MainWindow", u"Frames rendered/s: None", None))
        self.missed_frames.setText(
            QCoreApplication.translate(
                "MainWindow", u"Frames dropped/s: None", None))
        self.sensor_fps.setText(
            QCoreApplication.translate(
                "MainWindow", u"Frames sensed/s: None", None))
    # retranslateUi

    # =========================================================================

    def sliderToolTip(self, value):
        """
        :Description: Function updates the value for Horizontal slider and Spin box
        """
        self.horizontalSlider.setValue(value)
        self.spinBox.setValue(value)

    # =========================================================================

    def toggletoolbar(self, state):
        """
        :Description: Function enables the Dock Window
        """
        if state:
            self.dockWidget_2.show()
        else:
            self.dockWidget_2.close()
    # =========================================================================

    def openglLoaded(self):
        """
        :Description: Switch centralStack from Loading label to OpenGL window
        """
        self.centralStack.setCurrentIndex(1)

# =============================================================================
# =============================================================================

class Update_dockwidget_dynamic(QListWidget):
    dock_widget_sig = Signal()

    def resizeEvent(self, event):
        """
        :Description: Function to resize the dock widget 
        :Params: event
        :return: QtCore Signal
        """
        self.dock_widget_sig.emit()

# =============================================================================
# =============================================================================

class About_MessageBox(QMessageBox):
    """
    :Description: Function to resize the About Message Box 
    :Params: event
    :return: Message box geometry
    """
    def __init__(self):
        QMessageBox.__init__(self)

    def event(self, e):
        result = QMessageBox.event(self, e)
        self.setMinimumHeight(200)
        self.setMaximumHeight(16777215)
        self.setMinimumWidth(500)
        self.setMaximumWidth(16777215)
        self.setSizePolicy(QSizePolicy.Expanding, 
            QSizePolicy.Expanding)

        textEdit = self.findChild(QTextEdit)
        if textEdit != None :
            textEdit.setMinimumHeight(0)
            textEdit.setMaximumHeight(16777215)
            textEdit.setMinimumWidth(0)
            textEdit.setMaximumWidth(16777215)
            textEdit.setSizePolicy(QSizePolicy.Expanding, 
                QSizePolicy.Expanding)
        return result

# =============================================================================
# =============================================================================

def about_application(self):
    """
    :Description: Function Describes about the license 
    :return: Message Box with license Information is shown 
    """
    msg = About_MessageBox()
    msg.setIcon(QMessageBox.Information)
    msg.setStyleSheet(
            u"color: rgb(255, 255, 255); background-color: rgb(46, 52, 54);")
    # msg.move(self.centralwidget.rect().center())
    msg.setText(
        "MKE Point Cloud Viewer v" + __version__ +
        "                      ")
    msg.setInformativeText(
        "Utility for visualizing MagikEye sensor's point cloud data "
        "Copyright © MagikEye Inc. 2020-2021 under BSD-3-Clause licence.\n\n"
        "Using Pyside2 library licensed under GNU LGPL v2.1 \n")

    msg.setWindowTitle("About MKE Point Cloud Viewer")
    msg.setDetailedText(
        "Redistribution and use in source and binary forms, with or without "
        "modification, are permitted provided that the following conditions are met: \n"
        "\n"
        "1. Redistributions of source code must retain the above copyright notice, this"
        " list of conditions and the following disclaimer.\n"
        "\n"
        "2. Redistributions in binary form must reproduce the above copyright notice,"
        " this list of conditions and the following disclaimer in the documentation "
        "and/or other materials provided with the distribution.\n"
        "\n"
        "3. Neither the name of the copyright holder nor the names of its contributors"
        " may be used to endorse or promote products derived from this software "
        "without specific prior written permission.\n"
        "\n"
        "THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND "
        "ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED "
        "WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE "
        "DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE "
        "FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL "
        "DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR "
        "SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER "
        "CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, "
        "OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE "
        "OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
        "\n"
        "----\n"
        "\n"
        "Qt is a C++ toolkit for cross-platform application development.\n"
        "\n"
        "Qt provides single-source portability across all major desktop operating "
        "systems. It is also available for embedded Linux and other embedded and mobile "
        "operating systems.\n"
        "\n"
        "Qt is available under three different licensing options designed to accommodate "
        "the needs of our various users.\n"
        "\n"
        "Qt licensed under our commercial license agreement is appropriate for "
        "development of proprietary/commercial software where you do not want to share "
        "any source code with third parties or otherwise cannot comply with the terms of "
        "the GNU LGPL version 3.\n"
        "\n"
        "Qt licensed under the GNU LGPL version 3 is appropriate for the development of "
        "Qt applications provided you can comply with the terms and conditions of the "
        "GNU LGPL version 3.\n"
        "\n"
        "Please see qt.io/licensing for an overview of Qt licensing.\n"
        "\n"
        "Copyright (C) 2018 The Qt Company Ltd and other contributors.\n"
        "\n"
        "Qt and the Qt logo are trademarks of The Qt Company Ltd.\n"
        "\n"
        "Qt is The Qt Company Ltd product developed as an open source project. See qt.io "
        "for more information.\n\n")

    msg.exec_()

