#!/usr/bin/env python3

import os
from re import L
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget,QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import QThread
from std_msgs.msg import Empty
from colorama import Fore
from .kinova_joint_widget import KinovaJointWidget
from kinova_msgs.msg import JointAngles, JointVelocity
from kinova_msgs.srv import Start, Stop
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Transform, Pose
from std_msgs.msg import Header
import moveit_msgs.msg

from .inverse_kinematics import KinovaInverseKinematics
from geometry_msgs.msg import Transform
import math
import numpy as np


class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('kinova_gui')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)
        print("in my_module")
        # Create QWidget

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('kinova_gui'), 'resource', 'kinova_main.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('KinovaPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        
        context.add_widget(self._widget)

        # initiate the combo_box
        self._widget.control_mode_combo_box.addItem("Position Control",0)
        self._widget.control_mode_combo_box.addItem("Velocity Control",1)
        self._widget.control_mode_combo_box.addItem("Cartesian Control",2)
        self._widget.control_mode_combo_box.addItem("Trajectory Control",3)


        self._widget.control_mode_combo_box.currentIndexChanged.connect(self._control_mode_changed)
        self.control_mode = 0 
        #0 for indvidual joint control
        #1 for all joints control

        self.kinova_goal_topic = "kinova/goal"
        self.kinova_velocity_topic  = "/j2n6s300_driver/in/joint_velocity"
        self.kinova_goal_publisher = rospy.Publisher(self.kinova_goal_topic,Empty, queue_size=10)
        self.kinova_velocity_publisher = rospy.Publisher(self.kinova_velocity_topic,JointVelocity, queue_size=10)
        self.publish_rate = 100
        rospy.Timer(rospy.Duration(1/self.publish_rate), self._publish_kinova_command)

        self.kinova_command_angles = [0.0 for i in range(7)]
        self.joint_widgets = []

        self._widget.update_no_of_joints_button.pressed.connect(self._update_no_of_joints)

        self._widget.start_button.pressed.connect(self._start_button_pressed)
        self._widget.stop_button.pressed.connect(self._stop_button_pressed)
        self._widget.connect_button.pressed.connect(self._connect_button_pressed)
        self._widget.disconnect_button.pressed.connect(self._disconnect_button_pressed)


        #cartesian control
        self.initialx = 0
        self.initialy = 0.4

        self.x = self.initialx
        self.y = self.initialy
        self.ikSolver = KinovaInverseKinematics()
        self.query_transform = Transform()

        self._widget.up_button.pressed.connect(self._up_button_pressed)
        self._widget.down_button.pressed.connect(self._down_button_pressed)
        self._widget.right_button.pressed.connect(self._right_button_pressed)
        self._widget.left_button.pressed.connect(self._left_button_pressed)
        self._widget.vertical_button.pressed.connect(self._vertical_button_pressed)
        self._widget.update_trajectory_button.pressed.connect(self._update_trajectory_button_pressed)

        self._widget.x_double_spin_box.setMaximum(0.4)
        self._widget.x_double_spin_box.setMinimum(-0.4)
        self._widget.x_double_spin_box.setSingleStep(0.01)
        self._widget.x_double_spin_box.valueChanged.connect(self._x_changed)
        self._widget.x_double_spin_box.setValue(0.0)

        self._widget.y_double_spin_box.setMaximum(0.4)
        self._widget.y_double_spin_box.setMinimum(0.0)
        self._widget.y_double_spin_box.setSingleStep(0.01)
        self._widget.y_double_spin_box.valueChanged.connect(self._y_changed)
        self._widget.y_double_spin_box.setValue(0.4)
        self.way_points=[]

        self.curr_joint_angle_topic = "/j2n6s300_driver/out/joint_angles"
        self.curr_joint_angles = [0.0 for i in range(7)]
        self.curr_angle_subscriber = rospy.Subscriber(self.curr_joint_angle_topic, JointAngles, self.curr_angle_callback)
        self.goal_angles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.error_threshold = 1
        self.joint_velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_velocities_msg = JointVelocity()
        self.joint_speed = -5
        self.no_of_joints = 0
        self.rate = rospy.Rate(100)
        self.stop_publish  = True
        self.is_start = False
        self.display_path_publisher = rospy.Publisher("/kinova_cartesian_path", Path, queue_size=10)



    def curr_angle_callback(self, data):
        self.curr_joint_angles[0] = data.joint1
        self.curr_joint_angles[1] = data.joint2
        self.curr_joint_angles[2] = data.joint3
        self.curr_joint_angles[3] = data.joint4
        self.curr_joint_angles[4] = data.joint5
        self.curr_joint_angles[5] = data.joint6
        self.curr_joint_angles[6] = data.joint7
        print(self.curr_joint_angles)
        for i in range(self.no_of_joints):
            self.curr_joint_angles[i] = self.curr_joint_angles[i] - 360*(self.curr_joint_angles[i]//360) 
            self.joint_widgets[i]._widget.curr_angle_line_edit.setText(str(self.curr_joint_angles[i]))
        
        self._update_velocity()

    def _update_no_of_joints(self):
        # for i in range(self.no_of_joints):
        #     self._widget.controlJointsVLayout.removeWidget(self.joint_widgets[i]._widget)
        # self.clearlayout(self._widget.controlJointsVLayout)
        self.joint_widgets = []
        self.no_of_joints = self._widget.no_of_joints_spin_box.value()
        self.kinova_command_angles = [0.0 for i in range(self.no_of_joints)]
        print("in update number of joints")
        for i in range(self._widget.no_of_joints_spin_box.value()):
            self.add_kinova_joint_widget(i)

        print(self.kinova_command_angles)

    def clearlayout(self,layout):
        for i in reversed(range(layout.count())):
            print(layout.itemAt(i))
            layout.removeItem(layout.itemAt(i))

    def _kinova_joint_value_slider_changed(self):
        print("Slider changed")
        for i in range(self.no_of_joints):
            self.kinova_command_angles[i] = self.joint_widgets[i]._widget.joint_angle_slider.value()
            self.joint_widgets[i]._widget.command_angle_spin_box.setValue(self.kinova_command_angles[i]) 

    def _kinova_joint_value_spin_box_changed(self):
        print("stuck in loop")
        for i in range(self.no_of_joints):
            self.kinova_command_angles[i] = self.joint_widgets[i]._widget.command_angle_spin_box.value()
            self.joint_widgets[i]._widget.joint_angle_slider.setValue(self.kinova_command_angles[i])
    
    def add_kinova_joint_widget(self, joint_number):
        self.joint_widgets.append(KinovaJointWidget(joint_number))
        self.joint_widgets[joint_number]._widget.joint_angle_slider.valueChanged.connect(self._kinova_joint_value_slider_changed)
        self.joint_widgets[joint_number]._widget.curr_angle_line_edit.setReadOnly(True)
        self.joint_widgets[joint_number]._widget.command_angle_spin_box.valueChanged.connect(self._kinova_joint_value_spin_box_changed)

        self._widget.controlJointsVLayout.addWidget(self.joint_widgets[joint_number]._widget)
        self.joint_widgets[joint_number]._widget.joint_name_label.setText("joint_"+str(joint_number))


    def _control_mode_changed(self):
        print("control mode has changed")
        self.control_mode = self._widget.control_mode_combo_box.currentIndex()
        self._reset_sliders_n_spinboxes()
    
    def _reset_sliders_n_spinboxes(self):
        if self.control_mode == 0:
            #position control
            print("position control")
            for i in range(self.no_of_joints):
                self.joint_widgets[i]._widget.joint_angle_slider.setMaximum(360)
                self.joint_widgets[i]._widget.joint_angle_slider.setMinimum (-360)
                self.joint_widgets[i]._widget.command_angle_spin_box.setMaximum(360)
                self.joint_widgets[i]._widget.command_angle_spin_box.setMinimum(-360)
        elif self.control_mode == 1:
            #velocity_control
            print("velocity control")
            for i in range(self.no_of_joints):
                self.joint_widgets[i]._widget.joint_angle_slider.setMaximum(60)
                self.joint_widgets[i]._widget.joint_angle_slider.setMinimum(-60)
                self.joint_widgets[i]._widget.command_angle_spin_box.setMaximum(60)
                self.joint_widgets[i]._widget.command_angle_spin_box.setMinimum(-60)
        if self.control_mode == 2:
            #position control
            print("Cartessian control")
            for i in range(self.no_of_joints):
                self.joint_widgets[i]._widget.joint_angle_slider.setMaximum(360)
                self.joint_widgets[i]._widget.joint_angle_slider.setMinimum (-360)
                self.joint_widgets[i]._widget.command_angle_spin_box.setMaximum(360)
                self.joint_widgets[i]._widget.command_angle_spin_box.setMinimum(-360)
    
    def get_display_trajectories(self,traj):
        # for left leg
        display_traj = Path()
        display_traj.header = Header()
        display_traj.header.frame_id = "world" 
        display_traj.header.stamp = rospy.Time()
        display_traj.header.seq = 0
        display_traj.poses = []

        display_traj.poses = [PoseStamped() for i in range(len(traj))]
        for i in range(len(traj)):
            # defining the header for pose stamped
            display_traj.poses[i].header.frame_id = "world" 
            display_traj.poses[i].header.seq = 0
            display_traj.poses[i].header.stamp = rospy.Time()

            # setting the poses with respect to the world frame 
            display_traj.poses[i].pose = Pose()
            display_traj.poses[i].pose.position.x = 0.15
            display_traj.poses[i].pose.position.y = traj[i][0]
            display_traj.poses[i].pose.position.z = traj[i][1]
            display_traj.poses[i].pose.orientation.w = 1
        return(display_traj)

    def _play_button_pressed(self):
        print("executing joints")
        self.stop_publish = False
    
    def _stop_button_pressed(self):
        self.stop_publish = True
        self._initiate_joint_angles()


    def _start_button_pressed(self):
        if not self.is_start:
            self._widget.start_button.setText("Pause")
            self.stop_publish = False
            self.is_start = True
        else:
            self._widget.start_button.setText("Start")
            self.stop_publish = True
            self.is_start = False


    def _disconnect_button_pressed(self):
        #stop trajectories
        print("disconnecting")
        self.stop_publish = True
        self._call_stop_service()


    def _connect_button_pressed(self):
        self.stop_publish = True
        print("connecting")
        self._call_start_service()

    def _update_trajectory_button_pressed(self):
        input_string = self._widget.trajectory_text_edit.toPlainText()
        self.way_points = []
        xys = input_string.split("\n")
        self.trajectory_index = 0
        # self._widget.trajectory_text_edit.setReadOnly(True)
        for xy in xys:
            temp = xy.split(" ")
            self.way_points.append((float(temp[0]), float(temp[1])))
        self.interpolated_waypoints = []
        y_interp = []
        x_interp = []
        interpolation_length = 10
        for i in range(len(self.way_points) - 1):
            y_interp = list(np.linspace(self.way_points[i][1], self.way_points[i+1][1], interpolation_length))
            x_interp = list(np.linspace(self.way_points[i][0], self.way_points[i+1][0], interpolation_length))
            for i in range(interpolation_length):
                self.interpolated_waypoints.append((x_interp[i], y_interp[i]))

        self.way_points = self.interpolated_waypoints
        self.display_path_publisher.publish(self.get_display_trajectories(self.way_points))

    def _call_stop_service(self):
        # rospy.wait_for_service('/j2n6s300_driver/in/stop')
        try:
            stop_kinova = rospy.ServiceProxy('/j2n6s300_driver/in/stop', Stop())
            resp1 = stop_kinova()
            print(resp1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def _call_start_service(self):
        # rospy.wait_for_service('/j2n6s300_driver/in/start')
        try:
            start_kinova = rospy.ServiceProxy('/j2n6s300_driver/in/start', Start())
            resp1 = start_kinova()
            print(resp1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        
    
    def _initiate_joint_angles(self):
        for i in range(self.no_of_joints):
            self.joint_widgets[i]._widget.joint_angle_slider.setValue(self.curr_joint_angles[i])
            self.joint_widgets[i]._widget.command_angle_spin_box.setValue(self.curr_joint_angles[i])



    def _publish_kinova_command(self, event):
        # self.kinova_goal_publisher.publish(self.kinova_command_angles)
        self._update_velocity()
        if not self.stop_publish:
            self.kinova_velocity_publisher.publish(self.joint_velocities_msg)
            # print("publishing")


    def _update_velocity(self):
        
        if self.control_mode == 0:
            for i in range(self.no_of_joints):
                if self.kinova_command_angles[i] < 0:
                    self.kinova_command_angles[i] = self.kinova_command_angles[i] + 360
                error = self.kinova_command_angles[i] - self.curr_joint_angles[i]
                if error < 0:
                    vel = -self.joint_speed
                    if error <= -180:
                        vel = self.joint_speed
                        error = error + 360
                elif error > 0:
                    vel = self.joint_speed
                    if error >= 180:
                        vel = -self.joint_speed
                        error = error - 360

                if abs(error) < self.error_threshold:
                    vel = 0
                
                self.joint_velocities[i] = -vel
        
        elif self.control_mode == 1:
            for i in range(self.no_of_joints):
                self.joint_velocities[i] = self.joint_widgets[i]._widget.joint_angle_slider.value()
        
        elif self.control_mode == 2:
            self.query_transform.translation.y =self.y
            self.query_transform.translation.x =self.x
            angles = self.ikSolver.get_angles(self.query_transform)
            # print(math.degrees(angles[0]), math.degrees(angles[1]))
            initial_angles = [ 150, 180]
            theta1 = initial_angles[0] + math.degrees(angles[0])
            theta2 = initial_angles[1] - math.degrees(angles[1])
            self.kinova_command_angles[0] = theta1
            self.kinova_command_angles[1] = theta2
            # print(self.kinova_command_angles)
            for i in range(self.no_of_joints):
                if self.kinova_command_angles[i] < 0:
                    self.kinova_command_angles[i] = self.kinova_command_angles[i] + 360
                error = self.kinova_command_angles[i] - self.curr_joint_angles[i]
                if error < 0:
                    vel = -self.joint_speed
                    if error <= -180:
                        vel = self.joint_speed
                        error = error + 360
                elif error > 0:
                    vel = self.joint_speed
                    if error >= 180:
                        vel = -self.joint_speed
                        error = error - 360

                if abs(error) < self.error_threshold:
                    vel = 0
                
                self.joint_velocities[i] = -vel     

        elif self.control_mode == 3 and not self.stop_publish:
            if(len(self.way_points) == 0 or self.trajectory_index == len(self.way_points)):
                    return
            self.query_transform.translation.x =self.way_points[self.trajectory_index][0]
            self.query_transform.translation.y =self.way_points[self.trajectory_index][1]
            angles = self.ikSolver.get_angles(self.query_transform)
            # print(math.degrees(angles[0]), math.degrees(angles[1]))
            initial_angles = [ 150, 180]
            theta1 = initial_angles[0] + math.degrees(angles[0])
            theta2 = initial_angles[1] - math.degrees(angles[1])
            self.kinova_command_angles[0] = theta1
            self.kinova_command_angles[1] = theta2
            # print(self.kinova_command_angles)
            success_flag = True
            for i in range(self.no_of_joints):
                if self.kinova_command_angles[i] < 0:
                    self.kinova_command_angles[i] = self.kinova_command_angles[i] + 360
                error = self.kinova_command_angles[i] - self.curr_joint_angles[i]
                if error < 0:
                    vel = -self.joint_speed
                    if error <= -180:
                        vel = self.joint_speed
                        error = error + 360
                elif error > 0:
                    vel = self.joint_speed
                    if error >= 180:
                        vel = -self.joint_speed
                        error = error - 360

                if abs(error) < self.error_threshold:
                    success_flag = success_flag and True
                    vel = 0
                else:
                    success_flag = success_flag and False
                
                self.joint_velocities[i] = -vel          
            
            if success_flag:
                self.trajectory_index = self.trajectory_index + 1

            if self.trajectory_index == len(self.way_points):
                self.stop_publish = True
                        # self._widget.trajectory_text_edit.setText("Trajectory Completed!")
                        # self._widget.trajectory_text_edit.setReadOnly("False")
                

        
        self.joint_velocities_msg.joint1 = self.joint_velocities[0]
        self.joint_velocities_msg.joint2 = self.joint_velocities[1] 
        self.joint_velocities_msg.joint3 = self.joint_velocities[2] 
        self.joint_velocities_msg.joint4 = self.joint_velocities[3] 
        self.joint_velocities_msg.joint5 = self.joint_velocities[4] 
        self.joint_velocities_msg.joint6 = self.joint_velocities[5]
        self.joint_velocities_msg.joint7 = self.joint_velocities[6] 

    #cartesian control
    def _vertical_button_pressed(self):
        self.x = 0.0
        self.y = 0.4
        self._widget.y_double_spin_box.setValue(self.y)
        self._widget.x_double_spin_box.setValue(self.x)

    def _up_button_pressed(self):
        self.y = self.y + 0.01
        self._widget.y_double_spin_box.setValue(self.y)

    def _down_button_pressed(self):
        self.y = self.y - 0.01
        self._widget.y_double_spin_box.setValue(self.y)

    def _right_button_pressed(self):
        self.x = self.x + 0.01
        self._widget.x_double_spin_box.setValue(self.x)

    def _left_button_pressed(self):
        self.x = self.x - 0.01
        self._widget.x_double_spin_box.setValue(self.x)

    def _x_changed(self):
        self.x = self._widget.x_double_spin_box.value()
    
    def _y_changed(self):
        self.y = self._widget.y_double_spin_box.value()

    def _unregister_publisher(self):
        self.kinova_goal_publisher.unregister()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.stop_publish = True
        self._unregister_publisher()


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
