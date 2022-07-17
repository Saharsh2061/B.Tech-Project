#!/usr/bin/env python3

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from colorama import Fore

class KinovaJointWidget():
    def __init__(self, joint_number):
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('kinova_gui'), 'resource', 'joint_slider.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('joint_slider_widget' + str(joint_number))
        self._widget.joint_angle_slider.setMaximum(360)
        self._widget.joint_angle_slider.setMinimum(-360)
        self._widget.joint_angle_slider.setSingleStep(1)
        self._widget.joint_angle_slider.setValue(0)

        self._widget.command_angle_spin_box.setMaximum(360)
        self._widget.command_angle_spin_box.setMinimum(-360)
        self._widget.command_angle_spin_box.setSingleStep(1)
        self._widget.command_angle_spin_box.setValue(0)


