#!/usr/bin/env directorPython

# TODO: Make a simple camera viewing widget.

import argparse
import numpy as np

from vtk.util.numpy_support import vtk_to_numpy

from director import applogic
from director import consoleapp
from director import vtkAll as vtk
from director.timercallback import TimerCallback

import PythonQt
from PythonQt import QtGui

import math
import time

# This is more like director...CameraImageView than ImageWidget
class ImageWidget(object):
    def __init__(self):
        self.name = 'Image View'
        self.view = PythonQt.dd.ddQVTKWidgetView() #applogic.getViewManager().createView(self.name, 'VTK View')

        self.image = vtk.vtkImageData()
        self.initialized = False

        # Initialize the view.
        self.view.installImageInteractor()
        # Add actor.
        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.image)
        self.imageActor.SetVisibility(False)
        self.view.renderer().AddActor(self.imageActor)
        self.view.renderer().ResetCamera()

        self.view.orientationMarkerWidget().Off()
        self.view.backgroundRenderer().SetBackground(0,0,0)
        self.view.backgroundRenderer().SetBackground2(0,0,0)

        # Add timer.
        self.render_timer = TimerCallback(
            targetFps = 60,
            callback = self.render)
        self.start_time = time.time()
        self.render_timer.start()

    def update_image(self):
        w = 3
        h = 3
        num_components = 1

        image = vtk.vtkImageData()
        image.SetWholeExtent(0, w - 1, 0, h - 1, 0, 0)
        image.SetExtent(image.GetWholeExtent())
        image.SetSpacing(1., 1., 1.)
        image.SetOrigin(0.5, 0.5, 0.5)
        image.SetNumberOfScalarComponents(num_components)
        image.SetScalarType(vtk.VTK_UNSIGNED_CHAR)
        image.AllocateScalars()

        data = vtk_to_numpy(image.GetPointData().GetScalars())
        data.shape = image.GetDimensions()
        t = time.time() - self.start_time

        x, y = np.meshgrid(np.linspace(0., 1., w), np.linspace(0., 1., w))
        # value = (255 * (t / 1.)) % 256
        # data[:] = value
        s = t % 1.
        data[:, :, 0] = 255. * (x * s + y + (s - 1)) / 2.

        self.image.DeepCopy(image)

    def render(self):
        if not self.view.isVisible():
            return
        self.update_image()
        if not self.initialized:
            # Fit image to view.
            self.fit_image_to_view()

            # Ensure it is visible.
            self.imageActor.SetVisibility(True)
            self.initialized = True
        self.view.render()

    def fit_image_to_view(self):
        imageWidth, imageHeight = self.image.GetDimensions()[:2]
        viewWidth, viewHeight = self.view.renderWindow().GetSize()

        aspect_ratio = float(viewWidth) / viewHeight
        parallel_scale = max(imageWidth / aspect_ratio, imageHeight) / 2.0
        self.view.camera().SetParallelScale(parallel_scale)

class DrakeImageViewer(object):
    def __init__(self):
        self.create_window()

    def create_window(self):
        print "Creating"
        self.image_widgets = [ImageWidget()]

        # Create widget and layouts
        self.widget = QtGui.QWidget()
        self.layout = QtGui.QHBoxLayout(self.widget)
        for image_widget in self.image_widgets:
            self.layout.addWidget(image_widget.view)
        self.layout.setContentsMargins(0, 0, 0, 0)

        default_width = 640
        default_height = 480
        dim = [
            default_width * len(self.image_widgets),
            default_height]

        self.widget.resize(*dim)
        self.widget.show()

        print "Showing"

        # # Add shortcuts.
        # applogic.addShortcut(
        #     self.widget, 'Ctrl+Q', consoleapp.ConsoleApp.quit)
        # applogic.addShortcut(
        #     self.widget, 'F8', consoleapp.ConsoleApp.showPythonConsole)

if __name__ == "__main__":
    image_viewer = DrakeImageViewer()

    has_app = 'app' in globals()
    if not has_app:
        app = consoleapp.ConsoleApp()
        app.start()
