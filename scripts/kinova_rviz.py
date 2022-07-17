import rospy
from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointAngles
from std_msgs.msg import Header

def curr_angle_callback(data):
    curr_joint_angles[0] = data.joint1
    curr_joint_angles[1] = data.joint2
    curr_joint_angles[2] = data.joint3
    curr_joint_angles[3] = data.joint4
    curr_joint_angles[4] = data.joint5
    curr_joint_angles[5] = data.joint6
    curr_joint_angles[6] = data.joint7


    for i in range(7):
        curr_joint_angles[i] = curr_joint_angles[i] - 360*(curr_joint_angles[i]//360) 

    kinova_joint_state.position[0] = kinova_joint_offset[0] + curr_joint_angles[0]
    kinova_joint_state.position[1] = kinova_joint_offset[1] - curr_joint_angles[1]
    kinova_joint_state.position[2] = kinova_joint_offset[2] + curr_joint_angles[2]
    kinova_joint_state.position[3] = kinova_joint_offset[3] + curr_joint_angles[3]
    kinova_joint_state.position[4] = kinova_joint_offset[4] + curr_joint_angles[4]
    kinova_joint_state.position[5] = kinova_joint_offset[5] + curr_joint_angles[5]

    joint_state_publisher.publish(kinova_joint_state)

if __name__ == "__main__":
    rospy.init_node("rviz_kinova_vixualize", anonymous=True)

    curr_joint_angle_topic = "/j2n6s300_driver/out/joint_angles"
    curr_joint_angles = [0.0 for i in range(7)]
    curr_angle_subscriber = rospy.Subscriber(curr_joint_angle_topic, JointAngles, curr_angle_callback)
    joint_state_publisher = rospy.Publisher("joint_states", JointState, queue_size=10)
    kinova_joint_state = JointState()
    kinova_joint_state.header = Header()
    kinova_joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    kinova_joint_offset = [0.0, 222, -203, 0.0, 0.0, 0.0]
    kinova_joint_state.position = [0.0 for i in range(6)]
    kinova_joint_state.effort = [0.0 for i in range(6)]
    kinova_joint_state.velocity = [0.0 for i in range(6)]
    rospy.spin()
