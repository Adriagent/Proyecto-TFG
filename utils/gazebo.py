import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from time import time


class gazebo_controller:

    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    
    def __init__(self):
        # rospy.init_node("arm_controller")
        rospy.Subscriber('/joint_states', JointState, self.callback)
        self.pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

    def callback(self, data):
        feedback_q = list(data.position)
        feedback_q[0], feedback_q[2] = feedback_q[2], feedback_q[0] 
        self.q = feedback_q
    
    def move(self,q):
        movement = JointTrajectory()
        pts = JointTrajectoryPoint()

        movement.joint_names = self.joint_names
        pts.positions = q
        pts.velocities = [0]*6
        pts.accelerations = [0]*6
        pts.effort = [0]
        pts.time_from_start = rospy.Duration(0.01)    # Tiempo en el que se ha de realizar el movimiento
        movement.points = [pts]

        self.pub.publish(movement)

if __name__ == "__main__":
    rospy.init_node("arm_controller")
    arm = gazebo_controller()

    start = time()
    while time() - start < 0.5:
        arm.move([0]*6)