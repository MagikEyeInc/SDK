#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    Python pointcloud viewer application for MKE sensors
"""
__author__ = 'Sujay Jayashankar', 'Kiran NV', 'Karthik Raja S'
__copyright__ = 'Copyright (c) 2021, Magik-Eye Inc.'

# =============================================================================

from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from .viewer import Ui_MainWindow
from vispy import app, visuals, scene, color

from queue import Queue
import time
from datetime import datetime as dt
import re
import threading
import numpy as np
import copy
import sys
import os

import pymkeapi

os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
# =============================================================================
# =============================================================================


class SimpleViewer(QtWidgets.QMainWindow):
    handle_connection_sig = QtCore.Signal(str)
    stop_viewer_sig = QtCore.Signal()
    start_viewer_sig = QtCore.Signal()

    # =========================================================================

    def __init__(self):
        super(SimpleViewer, self).__init__()
        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.setWindowIcon(QIcon(os.path.join(current_dir,'icon/mke_logo.png')))
        self.settings = QSettings('MagikEye', 'PyMkeViewer')
        self.geometry = self.settings.value('geometry', bytes('', 'utf-8'))
        self.save_window = self.settings.value('savestate', bytes('', 'utf-8'))
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self)
        self.windowWidth = self.width()
        self.windowHeight = self.height()
        self._ui.widget = QWidget(self._ui.openGLWidget)
        self._ui.widget.setGeometry(QRect(
            0, 0, self.windowWidth, self.windowHeight))
        self._ui.widget.setLayout(QVBoxLayout())
        self._ui.pushButtonStart.clicked.connect(
            lambda: self.render_widget_action(
                self._ui.pushButtonStart.text()))
        self._ui.checkBoxAxis.stateChanged.connect(
            lambda: self._thread.check_axis(
                self._ui.checkBoxAxis.isChecked()))
        self._ui.dockWidget_2.toggleViewAction().toggled.connect(self.update_dock_widget)
        self._ui.dockWidgetContents_2.dock_widget_sig.connect(self.update_dock_widget)
        self._ui.pushButtonRefresh.clicked.connect(self.refresh_device_manager)
        self._ui.pushButtonConnect.clicked.connect(
            lambda: self.handle_connection(self._ui.pushButtonConnect.text()))
        self._ui.actionExit.triggered.connect(self.close_application)

        self._thread = Worker(self._ui.widget)
        self._ui.horizontalSlider.valueChanged.connect(
            self._thread.update_point)
        self._ui.comboBox.activated.connect(self._thread.update_color)
        self._thread.fps_text.connect(self._ui.fps.setText)
        self._thread.sensor_fps_text.connect(self._ui.sensor_fps.setText)
        self._thread.dfps_text.connect(self._ui.missed_frames.setText)
        self._thread.render_btn_text.connect(self._ui.pushButtonStart.setText)
        self._thread.disconnect_sig.connect(self.remote_connection)
        self._thread.opengl_loaded_sig.connect(self._ui.openglLoaded)

        self._remote_disconnection = False
        self.__disconnect = False
        self._radio_buttons = []
        self._ips = []
        self._selected_ip = None
        self._device_ids = []
        self._selected_device_id = None
        self.__connection_established = False
        self.handle_connection_sig.connect(self.handle_connection)
        self.start_viewer_sig.connect(self._thread.start_viewer)
        self.stop_viewer_sig.connect(self._thread.stop_viewer)

        self._msg = QMessageBox()
        self._msg.setWindowTitle("Connection Status")
        self._msg.setStyleSheet(
            u"color: rgb(255, 255, 255);\nbackground-color: rgb(46, 52, 54);")

        self.enable_color = u"color: rgb(255, 255, 255);\nbackground-color: rgb(46, 52, 54);"
        self.disable_color = u"color: rgb(0, 0, 0);\nbackground-color: rgb(186, 189, 182);"

        self._ui.pushButtonConnect.setDisabled(True)
        self._ui.pushButtonConnect.setStyleSheet(self.enable_color)
        self._ui.pushButtonStart.setDisabled(True)
        self._ui.checkBoxAxis.setDisabled(True)
        self._ui.horizontalSlider.setDisabled(True)
        self._ui.spinBox.setDisabled(True)
        self._ui.comboBox.setDisabled(True)

        self.save_point_size = self.settings.value("point_size", 
                                        self._ui.horizontalSlider.value(),  type=int)
        self.save_point_color = self.settings.value("point_color", 
                                        self._ui.comboBox.currentIndex(),  type=int)
        self.save_check_axis = self.settings.value("check_axis",
                                        self._ui.checkBoxAxis.isChecked(),  type=bool)
        self.read_config()
    
    # =========================================================================

    def read_config(self):
        """
        :Description: Function to restore previous settings.
        :Params: None 
        :return: None
        """
        try:
            self.restoreGeometry(self.geometry)
            self.restoreState(self.save_window)
            self.update_dock_widget()
            if self.save_point_size >= 0 and self.save_point_size <= 100:
                self._ui.horizontalSlider.setValue(self.save_point_size)
                self._ui.spinBox.setValue(self.save_point_size)
                self._thread.update_point(self.save_point_size)
            else:
                self._ui.horizontalSlider.setValue(5)
                self._ui.spinBox.setValue(5)
                self._thread.update_point(5)
            if self.save_point_color >= 0 and self.save_point_color <= 26:
                self._ui.comboBox.setCurrentIndex(self.save_point_color)
                self._thread.update_color(self.save_point_color)
            else:
                self._ui.comboBox.setCurrentIndex(0)
                self._thread.update_color(0)
            if self.save_check_axis:
                self._ui.checkBoxAxis.setChecked(True)
                self._thread.check_axis(True)
            else:
                self._ui.checkBoxAxis.setChecked(False)
                self._thread.check_axis(False)

        except Exception as msg:
            print(
                f"Exception occurred in read_config() due to :: {str(msg)}")
            print(f"Resetting to Default Configurations.")

    # =========================================================================

    def render_widget_action(self, state):
        """
        :Description: Function to handle/route the start and stop rendering action.
        :Params: state [str]: state can be either 'Start Rendering' or 'Stop Rendering'
        :return: None
        """
        try:
            if state == 'Start Rendering':
                self.start_viewer_sig.emit()
            elif state == 'Stop Rendering':
                self.stop_viewer_sig.emit()

        except Exception as msg:
            print(
                f"Exception occurred in render_widget_action() due to :: {str(msg)}")

    # =========================================================================
    @QtCore.Slot()
    def update_dock_widget(self):
        """
        :Description: Function to update height and width based on the dockable widget state.
        :Params: None
        :return: None
        """
        try:
            self._ui.horizontalSlider.setGeometry(QRect
                (30, 420, self._ui.dockWidget_2.width()-111, 20))
            self._ui.spinBox.setGeometry(QRect
                (self._ui.dockWidget_2.width()-70, 420, 48, 21))
            self._ui.pushButtonRefresh.setGeometry(QRect
                (self._ui.dockWidget_2.width()-110, 220, 100, 25))
            self._ui.pushButtonStart.setGeometry(QRect
                (30, 310, self._ui.dockWidget_2.width()-60, 25))
            self._ui.listView.setGeometry(QRect
                (10, 290, self._ui.dockWidget_2.width()-20, 210))
            self._ui.verticalLayoutWidget.setGeometry(QRect
                (10, 40, self._ui.dockWidget_2.width()-20, 170))
            self._ui.groupBox.setGeometry(QRect
                (0, 0, self._ui.dockWidget_2.width()-20, 170))
            self._ui.comboBox.setGeometry(QRect
                (120, 460, self._ui.dockWidget_2.width()-150, 25))

            if self._ui.dockWidget_2.isVisible() and not self._ui.dockWidget_2.isFloating():
                self.windowWidth = self.width() - self._ui.dockWidget_2.width()
                self.update_tool_box()
            else:
                self.windowWidth = self.width()
                self.update_tool_box()
            self.windowHeight = self.height()
            self._ui.widget.setGeometry(QRect(
                0, 0, self.windowWidth, self.windowHeight))

        except Exception as msg:
            print(
                f"Exception occurred in update_dock_widget() due to :: {str(msg)}")

    # =========================================================================

    def update_tool_box(self):
        """
        :Description: Function to update the Checkbox of the Toolbox button
        :Params: None
        :return: None
        """
        try:
            if self._ui.dockWidget_2.isVisible():
                self._ui.actionToolbox.setChecked(True)
            else:
                self._ui.actionToolbox.setChecked(False)

        except Exception as msg:
            print(
                f"Exception occurred in update_tool_box() due to :: {str(msg)}")

    # =========================================================================

    def update(self):
        """
        :Description: Function to update the width and height of window
        :Params: None
        :return: None
        """
        try:
            if self._ui.dockWidget_2.isFloating() or not self._ui.dockWidget_2.isVisible():
                self.windowWidth = self.width()
            else:
                self.windowWidth = self.width() - self._ui.dockWidget_2.width()
            self.windowHeight = self.height()
        except Exception as msg:
            print(f"Exception occurred in update() due to :: {str(msg)}")

    # =========================================================================

    def resizeEvent(self, event):
        """
        :Description: Function to adjust geometry on resizing the Application.
        :Params: event
        :return: None
        """
        try:
            self.update()
            self._ui.widget.setGeometry(QRect(
                0, 0, self.windowWidth, self.windowHeight))
        except Exception as msg:
            print(f"Exception occurred in resizeEvent() due to :: {str(msg)}")

    # =========================================================================

    def closeEvent(self, event):
        """
        :Description: Function to stop the rendering and disconnect the device on closing the Application from Control Button.
        :Params: event
        :return: None
        """
        self.close_application()

    # =========================================================================

    def close_application(self):
        """
        :Description: Function to stop the rendering and disconnect the device on closing the Application from Exit Button.
        :Params: event
        :return: None
        """
        try:
            if self._thread.isRunning():
                if verbose:
                    print("closeEvent -> stop thread")
                self._thread.stop()
            if self.__disconnect:
                if verbose:
                    print("closeEvent -> disconnect signal")
                self.handle_connection_sig.emit("Disconnect")
            self.geometry = self.saveGeometry()
            self.save_window = self.saveState()
            self.settings.setValue('geometry', self.geometry)
            self.settings.setValue("point_size", self._ui.horizontalSlider.value())
            self.settings.setValue("point_color", self._ui.comboBox.currentIndex())
            self.settings.setValue("check_axis", self._ui.checkBoxAxis.isChecked())
            self.settings.setValue("savestate", self.save_window)
            sys.exit()

        except Exception as msg:
            print(f"Exception occurred in closeEvent() due to :: {str(msg)}")
            
    # =========================================================================

    @QtCore.Slot()
    def refresh_device_manager(self):
        """
        :Description: Function to update the available devices in the DeviceManager GUI
        :Params: None
        :return: None
        """
        try:
            # Set connect button disabled. Will be enabled once the device selected.
            if not self.__connection_established:
                self._ui.pushButtonConnect.setDisabled(True)
                self._ui.pushButtonConnect.setStyleSheet(self.enable_color)

            if self._ui.pushButtonStart.text() == "Start Rendering":
                QApplication.setOverrideCursor(Qt.WaitCursor)
                # Disable the connect Button
                self._ui.pushButtonConnect.setEnabled(False)
                self._ui.pushButtonConnect.repaint()

                # Locate Devices
                devices = locate_devices()

                # Clear the existing devices from DeviceManager
                existing_rows = self._ui.formLayout.rowCount()
                for row in range(int(existing_rows)):
                    self._ui.formLayout.removeRow(0)

                # Add new devices
                self._radio_buttons.clear()
                self._ips.clear()
                radio_button = QRadioButton()
                if self._ui.pushButtonConnect.text() == "Disconnect":
                    if self._selected_ip:
                        self._radio_buttons.append(radio_button)
                        self._ips.append(self._selected_ip)
                        self._device_ids.append(self._selected_device_id)
                        label = QLabel(
                            self._selected_ip + " " + self._selected_device_id)
                        self._ui.formLayout.addRow(radio_button, label)
                        if self.__connection_established:
                            radio_button.setChecked(True)
                            radio_button.setDisabled(True)

                if len(devices) > 0:
                    for index, device in enumerate(devices):
                        radio_button = QRadioButton()
                        radio_button.clicked.connect(self.allow_connect)
                        ip, _id = device['ip'], device['id']
                        # Retain the state of selected ip if connection established
                        if ip != self._selected_ip:
                            self._radio_buttons.append(radio_button)
                            self._ips.append(ip)
                            self._device_ids.append(_id)
                            label = QLabel(ip + " " + _id)
                            self._ui.formLayout.addRow(radio_button, label)
                            self._ui.scrollArea.setWidget(self._ui.groupBox)
                            self._ui.verticalLayout_2.addWidget(
                                self._ui.scrollArea)
                        if self._ui.pushButtonConnect.text() == "Disconnect":
                            radio_button.setDisabled(True)

                else:
                    if len(self._ips) == 0:
                        QApplication.restoreOverrideCursor()
                        self._msg.move(self._ui.widget.rect().center())
                        self._msg.setIcon(QMessageBox.Information)
                        self._msg.setText("No Magik Eye device found on the local network.\n"
                                        "Please connect a Magik Eye device and try again.")
                        self._msg.exec_()

                # Enable the Connections
                if len(self._ips) == 0:
                    self._ui.pushButtonConnect.setEnabled(False)
                else:
                    self._ui.pushButtonConnect.setEnabled(True)

            else:
                self._msg.move(self._ui.widget.rect().center())
                self._msg.setIcon(QMessageBox.Warning)
                self._msg.setText("Stop rendering before refreshing")
                self._msg.exec_()
            
            QApplication.restoreOverrideCursor()
        except Exception as msg:
            QApplication.restoreOverrideCursor()
            print(
                f"Exception occurred in refresh_device_manager() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot(str)
    def remote_connection(self, action):
        """
        :Description: Function to handle disconnection action when MKE sensor Connection is Interrupted
        :Params: action [str]: action can be either Connect or Disconnect
        :return: None
        """
        self._remote_disconnection = True
        self.handle_connection(action)

    # =========================================================================

    @QtCore.Slot(str)
    def handle_connection(self, action):
        """
        :Description: Function to handle connection or disconnection actions
        :Params: action [str]: action can be either Connect or Disconnect
        :return: None
        """
        try:
            enable_color = u"color: rgb(255, 255, 255);\nbackground-color: rgb(46, 52, 54);"
            disable_color = u"color: rgb(0, 0, 0);\nbackground-color: rgb(186, 189, 182);"
            if action == "Connect":
                self.connect_to_device()
                if self._ui.pushButtonConnect.text() == "Disconnect":
                    self._ui.pushButtonStart.setDisabled(False)
                    self._ui.pushButtonStart.setStyleSheet(disable_color)
                    self._ui.horizontalSlider.setDisabled(False)
                    self._ui.spinBox.setDisabled(False)
                    self._ui.comboBox.setDisabled(False)
                    self._ui.comboBox.setStyleSheet(disable_color)
                    self._ui.checkBoxAxis.setDisabled(False)

            elif action == "Disconnect":
                self.disconnect_the_device()
                if self._ui.pushButtonConnect.text() == 'Connect':
                    self._ui.pushButtonStart.setDisabled(True)
                    self._ui.pushButtonStart.setStyleSheet(enable_color)
                    self._ui.horizontalSlider.setDisabled(True)
                    self._ui.spinBox.setDisabled(True)
                    self._ui.comboBox.setDisabled(True)
                    self._ui.checkBoxAxis.setDisabled(True)
                    self._ui.comboBox.setStyleSheet(enable_color)

        except Exception as msg:
            print(
                f"Exception occurred in handle_connection() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot()
    def allow_connect(self):
        """
        :Description: Function to enable the connect button only when a device selected in GUI
        :Params: None
        :return: None
        """
        self._ui.pushButtonConnect.setDisabled(False)
        self._ui.pushButtonConnect.setStyleSheet(self.disable_color)

    # =========================================================================

    @QtCore.Slot()
    def connect_to_device(self):
        """
        :Description: Function to handle the Device connection feature.
        :Params: None
        :return: None
        """
        # Establishing actual connection with the Sensor
        for index, radio in enumerate(self._radio_buttons):
            if radio.isChecked():
                self._selected_ip = self._ips[index]
                self._selected_device_id = self._device_ids[index]
                break

        info_msg = ""
        self._msg.move(self._ui.widget.rect().center())
        if self._selected_ip:
            try:
                connection_status = self._thread.connect_sensor(
                    self._selected_ip)
                if verbose:
                    print("IPs and Device IDs :", self._ips, self._device_ids)
                    print(f"Connecting to the ip = {self._selected_ip} ...")
                    print("CONNECTION RESULT: ", connection_status)
                if connection_status:
                    self.setWindowTitle(
                        "Connected to MKE Sensor " + str(self._selected_device_id) 
                        + "@" + str(self._selected_ip))
                    self.__disconnect = True
                    self.__connection_established = True
                    self._ui.pushButtonConnect.setText("Disconnect")
                    for index, radio in enumerate(self._radio_buttons):
                        if self._ips[index] == self._selected_ip:
                            radio.setChecked(True)
                            radio.setDisabled(True)
                        else:
                            radio.setDisabled(True)

                else:
                    self._msg.setIcon(QMessageBox.Warning)
                    info_msg = "Connection Failed"
                    self._msg.setText(info_msg)
                    self._selected_ip = None
                    self._msg.exec_()

            except Exception as msg:
                print(
                    f"Exception occurred in connect_to_device() due to :: {str(msg)}")

        else:
            info_msg = "Please select any device to connect"
            self._msg.setIcon(QMessageBox.Warning)
            self._msg.setText(info_msg)
            if verbose:
                print(info_msg)
            self._msg.exec_()

    # =========================================================================

    @QtCore.Slot()
    def disconnect_the_device(self):
        """
        :Description: Function to handle the Device disconnection feature.
        :Params: None
        :return: None
        """
        try:
            info_msg = ""
            if self._ui.pushButtonConnect.text() == "Disconnect":
                # Disconnect device
                self._msg.move(self._ui.widget.rect().center())
                if self._ui.pushButtonStart.text() == "Start Rendering":
                    self.setWindowTitle("PyMkEViewer")
                    self._thread.disconnect_sensor()
                    self.__connection_established = False
                    self._selected_ip = None
                    self._msg.setIcon(QMessageBox.Critical)
                    if self._remote_disconnection is True:
                        info_msg = "MKE sensor connection interrupted:" \
                                   "\ndevice has been disconnected"
                        self._remote_disconnection = False
                        self._msg.setText(info_msg)
                        self._msg.exec_()
                        self.refresh_device_manager()
                    self.__disconnect = False
                    self._ui.pushButtonConnect.setText("Connect")
                    for index, radio in enumerate(self._radio_buttons):
                        radio.setEnabled(True)
                        radio.setCheckable(True)
                else:
                    self._msg.setIcon(QMessageBox.Warning)
                    info_msg = "Stop rendering before disconnecting"
                    self._msg.setText(info_msg)
                    self._msg.exec_()
                if verbose:
                    print("Disconnecting device")
                    print(info_msg)

        except Exception as err_msg:
            print(
                f"Exception occurred in disconnect_the_device() due to :: {str(err_msg)} ")
            info_msg = "Unable to disconnect the device.\n"\
                "Please try again.\nMore Info:{" + str(err_msg) + "}"
            self._msg.setIcon(QMessageBox.Warning)
            self._msg.setText(info_msg)
            self._msg.exec_()

# =============================================================================


def locate_devices():
    """
    :Description: Function to locate the available Mke Sensors
    :Params: None
    :return: None
    """
    try:
        mke_sensors = []
        mke_discovery = pymkeapi.discover_devices()
        for key,value in mke_discovery.items():
            mke_sensors.append({'ip':value,'id':key})

        if verbose:
            print('MKE sensors: ')
            for mke_sensor in mke_sensors:
                print(mke_sensor['ip'], mke_sensor['id'])
        return mke_sensors
    
    except Exception as msg:
        print(f"Exception occured in locate_devices() due to :: {str(msg)}")
        QApplication.restoreOverrideCursor()
        msg_box = QMessageBox()
        msg_box.setStyleSheet(u"color: rgb(255, 255, 255);\n"
                          "background-color: rgb(46, 52, 54);")
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText(f"Unable to locate the devices!\nmore Info:{str(msg)}")
        msg_box.exec_()

# =============================================================================
# =============================================================================

class Worker(QtCore.QThread):
    end_sig = QtCore.Signal()
    model_updated = QtCore.Signal()
    fps_update = QtCore.Signal()
    fps_text = QtCore.Signal(str)
    dfps_text = QtCore.Signal(str)
    sensor_fps_text = QtCore.Signal(str)
    render_btn_text = QtCore.Signal(str)
    disconnect_sig = QtCore.Signal(str)
    opengl_loaded_sig = QtCore.Signal()

    # =========================================================================

    def __init__(self, widget, parent=None):
        super().__init__(parent)
        if verbose:
            print("INIT")
        self.sensor_port = 8888  # Port at which mkert exec is running
        self.__color_options = [
            'autumn', 'blues', 'cool', 'greens', 'reds', 'spring', 'summer',
            'fire', 'grays', 'hot', 'ice', 'RdYeBuCy', 'winter', 'light_blues',
            'orange', 'viridis', 'coolwarm', 'PuGr', 'GrBu', 'GrBu_d', 'RdBu',
            'cubehelix', 'single_hue', 'hsl', 'husl', 'diverging']
        self._col = color.get_colormap('autumn')

        self._widget = widget

        self._stream = None
        self.__bus = None
        self.__client = None
        self._frame = None
        self.__process = -1
        self._FPS = 0
        self._FDPS = 0
        self._fps_window_size = 5
        self._queue_size = 10
        self._sensor_fps = 0
        self.__seq_timer_list = list()
        self._missed_frames = 0
        self.__viewer_start_time = 0
        self._point_size = 5
        self._FRAME_TYPE = 1

        self.__axis_created = False 
        self.__axis_visibility = True

        self.__lock = threading.Lock()
        self._queue_buffer = Queue(self._queue_size)
        self._render_buffer = Queue(self._queue_size)
        self._fps_buffer = Queue(self._queue_size)
        self.__model = pymkeapi.Frame()

        self.__init_view_pyvis()

        scatter3d = scene.visuals.create_visual_node(visuals.MarkersVisual)
        self.__p1 = scatter3d(parent=self.__view.scene)
        self.__p1.set_gl_state('translucent', blend=True, depth_test=True)
        self._widget.layout().addWidget(self.__canvas.native)

        self.model_updated.connect(self.render_view)
        self.fps_update.connect(self.update_fps)
        self.end_sig.connect(self.stop_viewer)

    # =========================================================================

    def __init_view_pyvis(self):
        try:
            self.__canvas = scene.SceneCanvas(show=True)
            self.__view = self.__canvas.central_widget.add_view()
            self.__view.camera = 'turntable'
            self.__view.camera.fov = 45
            self.__view.camera.distance = 1
            self.__view.camera.up = 'z'
            if verbose:
                print("Center ", self.__view.camera.center)

            # Trivial XYZ axes lines that are initialized much faster:
            # self.__xyz = scene.XYZAxis(
            #     parent=self.__view.scene)

            QTimer.singleShot(200, self.__create_full_axis_once)  # run after 0.2s

        except Exception as msg:
            print(
                f"Exception occurred in __init_view_pyvis() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot(str)
    def __create_full_axis_once(self):

        if self.__axis_created:
            return
        self.__axis_created = True

        try:
            self.__xax = scene.Axis(
                pos=[[0, 0],
                     [1, 0]],
                tick_direction=(0, 1),
                domain=(0, 2000),
                axis_color='r', tick_color='r', text_color='r', font_size=0,
                tick_width=1, axis_width=1, axis_label="x",
                parent=self.__view.scene)
            self.__x_text = scene.Text(
                text=["X", "200", "400", "600", "800", "1000", "1200",
                      "1400", "1600", "1800", "2000"],
                pos=[(0.5, -0.02), (0.1, 0.02), (0.2, 0.02), (0.3, 0.02),
                     (0.4, 0.02), (0.5, 0.02), (0.6, 0.02), (0.7, 0.02),
                     (0.8, 0.02), (0.9, 0.02), (1, 0.02)],
                color='r', font_size=10,
                parent=self.__view.scene)
            self.__zero_text = scene.Text(
                text=["0"],
                pos=[(-0.01, -0.01)],
                color='r', font_size=10,
                parent=self.__view.scene)
            self.__yax = scene.Axis(
                pos=[[0, 0],
                     [0, 1]],
                tick_direction=(1, 0),
                domain=(0, 2000),
                axis_color='g', tick_color='g', text_color='g', font_size=0,
                tick_width=1, axis_width=1, axis_label="y",
                parent=self.__view.scene)
            self.__y_text = scene.Text(
                text=["Y", "200", "400", "600", "800", "1000", "1200",
                      "1400", "1600", "1800", "2000"],
                pos=[(-0.02, 0.5), (0.02, 0.1), (0.02, 0.2), (0.02, 0.3),
                     (0.02, 0.4), (0.02, 0.5), (0.02, 0.6), (0.02, 0.7),
                     (0.02, 0.8), (0.02, 0.9), (0.02, 1)],
                color='g', font_size=10,
                parent=self.__view.scene)
            self.__zax = scene.Axis(
                pos=[[0, 0],
                     [-1, 0]],
                tick_direction=(0, 1),
                domain=(0, 2000),
                axis_color='b', tick_color='b', text_color='b', font_size=0,
                tick_width=1, axis_width=1, axis_label="z",
                parent=self.__view.scene)
            self.__z_text = scene.Text(
                text=["Z", "200", "400", "600", "800", "1000", "1200",
                      "1400", "1600", "1800", "2000"],
                pos=[(-0.5, -0.02), (-0.1, 0.02), (-0.2, 0.02), (-0.3, 0.02),
                     (-0.4, 0.02), (-0.5, 0.02), (-0.6, 0.02), (-0.7, 0.02),
                     (-0.8, 0.02), (-0.9, 0.02), (-1, 0.02)],
                color='b', font_size=10,
                parent=self.__view.scene)
            # its actually an inverted xaxis
            self.__zax.transform = scene.transforms.MatrixTransform()
            self.__zax.transform.rotate(90, (0, 1, 0))  # rotate cw around yaxis
            # tick direction towards (-1,-1)
            self.__zax.transform.rotate(-45, (0, 0, 1))
            self.__z_text.transform = scene.transforms.MatrixTransform()
            self.__z_text.transform.rotate(90, (0, 1, 0))  # rotate cw around yaxis
            # tick direction towards (-1,-1)
            self.__z_text.transform.rotate(-45, (0, 0, 1))
            
            self.check_axis(self.__axis_visibility)

            self.opengl_loaded_sig.emit()

        except Exception as msg:
            print(
                f"Exception occurred in __create_full_axis_once() due to :: {str(msg)}")

    # =========================================================================

    def check_axis(self, state: bool):
        """
        :Description: Function to enable or disable the 3d axis
        :Params: state [boolean]
        :return: None
        """
        try:
            self.__axis_visibility = state
            if self.__axis_created:
                self.__xax.visible = state
                self.__yax.visible = state
                self.__zax.visible = state
                self.__x_text.visible = state
                self.__y_text.visible = state
                self.__z_text.visible = state
                self.__zero_text.visible = state

            if verbose:
                print("AXIS STATUS: ", state)

        except Exception as msg:
            print(f"Exception occurred in check_axis() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot(str)
    def connect_sensor(self, selected_ip):
        """
        :Description: Function to Establish a connection with the MKe Sensor
        :Params:  selected IP to get connected with
        :return: Boolean
        """
        try:
            # Go to state depth sensing
            self.__bus = pymkeapi.TcpBus(selected_ip, self.sensor_port)
            self.__client = pymkeapi.SyncClient(self.__bus)
            return True

        except Exception as msg:
            print(
                f"Exception occurred in connect_sensor() due to :: {str(msg)}")
            return False

    # =========================================================================

    @QtCore.Slot()
    def disconnect_sensor(self, force_stop=False):
        """
        :Description: Function to disconnect the connection between python viewer and Mke Sensor
        (Deletes the objects bus and client which holds the connection).
        :Params: None
        :return: None
        """
        try:
            del self.__bus
            del self.__client
        except Exception as msg:
            print(
                f"Exception occurred in disconnect_sensor() due to :: {str(msg)}")

    # =========================================================================

    def run(self):
        """
        :Description: Run function of the Qthread, handles the frame movement between the different queues
        :Params: None
        :return: None
        """
        try:
            if verbose:
                print("QTHREAD RUNNING")
            force_stop = False
            while self.__process == 0:
                if not self._queue_buffer.empty():
                    # Display current frame details on Frame Details widget
                    self._frame = copy.deepcopy(self._queue_buffer.get())
                    if not self._frame:
                        print(
                            "-" * 10,
                            "\nConnection Issue, Received Signal to Stop the Viewer\n",
                            "-" * 10)
                        force_stop = True
                        break
                    if not self._render_buffer.full():
                        self._render_buffer.put(self._frame)
                    if not self._fps_buffer.full():
                        self._fps_buffer.put(self._frame)
                    self.fps_update.emit()
                    self.model_updated.emit()
                    time.sleep(0.01)
                    self._queue_buffer.task_done()
            if force_stop:
                self.stop_viewer(force_stop=True)
                self.disconnect_sig.emit("Disconnect")

            if verbose:
                print("QTHREAD TERMINATING")

        except Exception as msg:
            print(f"Exception occurred in run() due to :: {str(msg)}")

    # =========================================================================

    def stop(self):
        """
        :Description: Function to manage closeEvent() by safely terminating the thread and Qthread
        :Params: None
        :return: None
        """
        try:
            if verbose:
                print("Is thread alive? ", self._stream.is_alive())
            if self._stream.is_alive():
                self.end_sig.emit()
            self.wait()
            if verbose:
                print("THREAD TERMINATION SUCCESSFUL")
        except Exception as msg:
            print(f"Exception occurred in stop() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot()
    def start_viewer(self):
        """
        :Description: Function to start the rendering process
        :Params: None
        :return: None
        """
        try:
            if verbose:
                print("START RENDERING")
            self.__viewer_start_time = dt.now()
            if self.__seq_timer_list:
                self.__seq_timer_list.clear()
            if self.__client.get_state() != pymkeapi.MKE_STATE_DEPTH_SENSOR:
                if verbose:
                    print("STATE CHANGE FROM ", self.__client.get_state())
                self.__client.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)
            if verbose:
                print("CURRENT STATE ", self.__client.get_state())
            self.__process = 0
            self._stream = threading.Thread(
                target=stream_acquire,
                args=(self._queue_buffer, self.__client, self.__lock))
            self._stream.setDaemon(True)
            self._stream.start()
            self.start()
            self.render_btn_text.emit("Stop Rendering")

        except Exception as msg:
            print(f"Exception occurred in start_viewer() due to :: {str(msg)}")
            self.disconnect_sig.emit("Disconnect")

    # =========================================================================

    @QtCore.Slot()
    def stop_viewer(self, force_stop=False):
        """
        :Description: Function to stop the rendering process
        :Params: force_stop (boolean)   [optional]
        :return: None
        """
        try:
            if verbose:
                print("STOP RENDERING")
            self.__lock.acquire()
            self.__process = 1
            self._stream.join()
            time.sleep(0.01)
            if not force_stop:
                frame = self.__client.get_frame(self._FRAME_TYPE)
                self.__client.set_state(pymkeapi.MKE_STATE_IDLE)
            self.__lock.release()
            if verbose:
                print("IS THREAD ALIVE? ", self._stream.is_alive())
            self.render_btn_text.emit("Start Rendering")
        except Exception as msg:
            print(f"Exception occurred in stop_viewer() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot(int)
    def update_point(self, value):
        """
        :Description: Function to update the points in viewer
        :Params: value
        :return: None
        """
        try:
            self._point_size = value
            if verbose:
                print("POINT SIZE CHANGED TO ", self._point_size)
            if self.__process == 1:
                self.model_updated.emit()

        except Exception as msg:
            print(f"Exception occurred in update_point() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot(str)
    def update_color(self, value):
        """
        :Description: Function to update the point color in viewer
        :Params: value [str] : value of color
        :return: None
        """
        try:
            if verbose:
                print("COLOR CHANGED TO ", self.__color_options[value])
            self._col = color.get_colormap(self.__color_options[value])
            if self.__process == 1:
                self.model_updated.emit()

        except Exception as msg:
            print(f"Exception occurred in update_color() due to :: {str(msg)}")

    # =========================================================================
    
    @QtCore.Slot()
    def update_fps(self):
        """
        :Description: Function to calculate Render FPS, Frame DPS,
        Sensor FPS and emit signal to display them in the Python viewer
        :Params: None
        :return: None
        """
        try:
            frames_missed = 0
            if self._frame:
                timer = self._frame.timer
                seqn = self._frame.seqn
                if len(self.__seq_timer_list) < self._fps_window_size:
                    self.__seq_timer_list.append((seqn, timer))
                else:
                    self.__seq_timer_list.remove(self.__seq_timer_list[0])
                    self.__seq_timer_list.append((seqn, timer))

                if len(self.__seq_timer_list) > 1:
                    seq_diff = int(
                        self.__seq_timer_list[-1][0]) - int(self.__seq_timer_list[0][0])
                    time_diff = int(
                        self.__seq_timer_list[-1][1]) - int(self.__seq_timer_list[0][1])
                    time_in_secs = time_diff / 1000

                    diff_bw_end_frames = int(
                        self.__seq_timer_list[-1][0]) - int(self.__seq_timer_list[-2][0])
                    if diff_bw_end_frames > 1:
                        frames_missed = diff_bw_end_frames - 1
                        self._missed_frames += frames_missed

                    # Determine FDPS
                    current_time = dt.now()
                    if current_time.second != self.__viewer_start_time.second:
                        if self._missed_frames:
                            self._FDPS = self._missed_frames / (
                                current_time.second - self.__viewer_start_time.second)
                        else:
                            self._FDPS = 0
                        if seq_diff:
                            self._FPS = seq_diff / time_in_secs
                        self.fps_text.emit("Frames rendered/s: " + str(int(self._FPS)))
                        self.dfps_text.emit(
                            "Frames dropped/s: " + str(int(self._FDPS)))
                        self.__viewer_start_time = current_time
                        self._missed_frames = 0
                        self._sensor_fps = self._FPS + float(self._FDPS)
                        self.sensor_fps_text.emit(
                            "Frames sensed/s: " + str(int(self._sensor_fps)))
        except Exception as msg:
            print(f"Exception occurred in update_fps() due to :: {str(msg)}")

    # =========================================================================

    @QtCore.Slot()
    def render_view(self):
        """
        :Description: Function to view the rendered data in 3d
        :Params: None
        :return: None
        """
        try:
            if self.__process == 0:
                self.__model = copy.deepcopy(self._render_buffer.get())

            points_length = self.__model.pts3d.shape[0]
            pts3d = self.__model.pts3d
            pts3d = pts3d[pts3d[:, 2].argsort()]
            pts3d = pts3d / 2000
            colormap = self._col[np.linspace(0., 1., points_length)]
            colors = colormap.rgba
            if points_length > 0:
                self.__p1.set_data(
                    pts3d, face_color=colors, symbol='o', size=self._point_size,
                    edge_width=0.5, edge_color='black')
        except Exception as msg:
            print(f"Exception occurred in render_view() due to :: {str(msg)}")

# =============================================================================

def stream_acquire(q, client, lock):
    """
    :Description: Function to get frame from the client object and add the frame in the queue buffer.
    :Params: client [object], lock
    :return: None
    """
    try:
        while True:
            if not lock.locked():
                frame = client.get_frame(1)
                q.put(frame)
            else:
                if verbose:
                    print("Lock set, entering termination mode for thread")
                break
            time.sleep(0.0001)

    except Exception as msg:
        print(f"Exception occurred in stream_acquire() due to :: {str(msg)}")
        # Disconnect the Device
        q.put(False)


# =============================================================================
# =============================================================================

def main_func():
    App = QApplication([])
    viewer = SimpleViewer()
    viewer.show()
    app.run()

# =============================================================================

sys_args = sys.argv[1:]
verbose = "-v" in sys_args

if __name__ == '__main__':
    main_func()
