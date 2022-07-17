# kinova_gui

## Installation

- clone the [kinova_ros](https://github.com/Kinovarobotics/kinova-ros) package in your workspace.
- clone this package.
- build it in your workspace.

## GUI
![GUI](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/images/labelled_image.png)


## Usage
- Use `rosrun kinova_gui kinova_gui`
- The gui should appear like this.
  ![Initial gui](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/images/initial_gui.png)
- Enter the no of joint to be used and press update.
  ![set noof joints](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/images/set_noof_joints.png)
- You should see two joint sliders at the bottom of the GUI.
  ![after adding joints](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/images/should_appear_like_this.png)
- Use the combo box to select the control mode.
- 
  ![control mode select](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/images/control_mode_selection.png)
- This text field is used to input the trajectory, press update trajectory after inputing the trajectory.
  - use space between x and y, each waypoint must be entered in new line.   
  ![Trajectory input](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/images/trajectory_input.png)

## Example Video
Download and check the example video. Robot draws a rectangle.
![example video](https://github.com/PrajwalSangam1310/kinova_gui/blob/main/video/example_video.mp4)

