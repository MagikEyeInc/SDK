#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This sample uses Pyside2 and Vispy to present realtime detections from sensor
"""

__author__ = "Ondra Fisar", "Sujay Jayashankar"
__copyright__ = "Copyright (c) 2019-2020, Magik-Eye Inc."

# -----------------------------------------------------------------------------

from PySide2 import QtCore, QtGui, QtWidgets
from vispy import app, visuals, scene, color, geometry
import numpy as np
import copy
import threading
import sys

import pymkeapi


# -----------------------------------------------------------------------------

class SimpleViewer(QtWidgets.QMainWindow):
    FRAME_TYPE = 1
    MAX_DIST = 2000
    GRID_SCALE = 0.01
    POINT_SIZE = 5

    model_updated = QtCore.Signal()
    critical_occured = QtCore.Signal(str)

    def __init__(self):
        super().__init__()

        self.__win = self
        self.__model = pymkeapi.Frame()

        self.__shared = threading.Lock()
        self.__init_ui()

        self.__running = False
        if len(sys.argv) > 1:
            self.__host = (str(sys.argv[1]), 8888)
        else:
            self.__host = self.__get_host()
        if self.__host is None:
            QtWidgets.QApplication.exit()

        self.__thread = threading.Thread(target=self.__thread_proc)
        self.__thread.start()

    def closeEvent(self, event):
        self.__running = False

    def warning(self, msg):
        print("WARNING: " + msg)

    def critical(self, msg):
        QtWidgets.QMessageBox.critical(self, "Error", msg)
        self.close()

    def __init_view(self):
        canvas = scene.SceneCanvas(show=True)
        view = canvas.central_widget.add_view()
        view.camera = 'turntable'
        view.camera.fov = 45
        view.camera.distance = 2
        view.camera.up = 'z'
        position = np.zeros((10,3), dtype=np.float32)
        position[:, 0] = np.linspace(0.0, 1.0, 10)
        position[:, 1] = np.linspace(0.0, 1.0, 10)
        col = color.Color((0,0,0,0), alpha=0.0)
        plane = scene.visuals.Plane(
            width=1, height=1,
            width_segments=10,
            height_segments=10,
            direction='+z',
            color=col,
            edge_color='w',
            parent=view.scene)
        return view, canvas

    def __init_ui(self):
        self.__view, self.__canvas = self.__init_view()
        self.__main_window = QtWidgets.QWidget()
        self.col = color.get_colormap('fire')
        self.__win.setCentralWidget(self.__canvas.native)
        scatter3d = scene.visuals.create_visual_node(visuals.MarkersVisual)
        self.__points = scatter3d(parent=self.__view.scene)
        self.__points.set_gl_state('translucent', blend=True, depth_test=True)
        self.model_updated.connect(self.render_view)
        self.critical_occured.connect(self.critical)

        # self.__init_toolbar()

    def __get_host(self):
        host, ok = QtWidgets.QInputDialog.getText(self.__win, 'Connect to sensor',
                                                  'Host')

        if ok and not host is None and host != '':
            return (host, 8888)

        return None

    def connect_to_sensor(self, host, port):
        bus = pymkeapi.TcpBus(host, port)
        self.__client = pymkeapi.SyncClient(bus)

        # go to state depth sensing

        state = self.__client.get_state()

        if state != pymkeapi.MKE_STATE_DEPTH_SENSOR:

            self.warning("Invalid state, let's change it to DEPTH_SENSOR")

            # firstly set state to IDLE

            if state != pymkeapi.MKE_STATE_IDLE:
                self.__client.set_state(pymkeapi.MKE_STATE_IDLE)

    def start_pushing(self):
        self.__start_seq_id = self.__client.start_frame_push(SimpleViewer.FRAME_TYPE)
        self.__stop_seq_id = None

    def stop_pushing(self):
        self.__stop_seq_id = self.__client.stop_frame_push()

    def __thread_proc(self):
        try:
            self.__running = True
            self.connect_to_sensor(self.__host[0], self.__host[1])

            # then change state to DEPTH_SENSOR

            if self.__client.get_state() != pymkeapi.MKE_STATE_IDLE:
                self.__client.set_state(pymkeapi.MKE_STATE_IDLE)
            self.__client.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)

            self.start_pushing()
            while self.__running:
                frame = self.__client.get_pushed_frame(self.__start_seq_id, self.__stop_seq_id)
                if frame is None:
                    break

                with self.__shared:
                    self.__model = copy.deepcopy(frame)
                self.model_updated.emit()

            self.__stop_seq_id = self.stop_pushing()
            raise RuntimeError("Data stream from host has been interupted")

        except Exception as e:
            self.__runnning = False
            self.critical_occured.emit(str(e))
        finally:
            try:
                self.__client.set_state(pymkeapi.MKE_STATE_IDLE)
            except:
                pass

    @QtCore.Slot()
    def render_view(self):
        with self.__shared:
            N = self.__model.pts3d.shape[0]
            pts3d = self.__model.pts3d
            pts3d = pts3d[pts3d[:,2].argsort()]
            pts3d = pts3d / SimpleViewer.MAX_DIST
            colormap = self.col[np.linspace(0., 1., N)]
            colors = colormap.rgba
            if N > 0:
                self.__points.set_data(
                    pts3d, face_color=colors, symbol='o',
                    size=SimpleViewer.POINT_SIZE, edge_width=0.5,
                    edge_color='black')


# -----------------------------------------------------------------------------

if __name__ == '__main__':
    App = QtWidgets.QApplication([])
    viewer = SimpleViewer()
    viewer.show()
    app.run()

# -----------------------------------------------------------------------------
