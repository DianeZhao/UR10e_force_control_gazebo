#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import pinocchio as pin
#from .parameter import Mlist, Glist, Slist
# from ur10e_force_msgs.msg import ForceCmd  # Assuming this message exists
import modern_robotics as mr  # Must be installed separately
from geometry_msgs.msg import PoseStamped

JOINT_SIZE = 6
gravity = np.array([0.0, 0.0, -9.8])
urdf_path = "/home/danningzhao/modern_robotics_ws/src/UR10e_force_control_gazebo/ur10e_description/urdf/ur10e.urdf"
FRAME_NAME = "ee_link"


class ForceControlClientSubscriber:
    def __init__(self):
        
        #model = pin.buildModelsFromUrdf(urdf_path) returns a tuple, kinematic model, collsision model and visual model
        #########################USE PINnOCHIO#######################################
        self.model = pin.buildModelFromUrdf(urdf_path) #only kinematic
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(FRAME_NAME)
        
        self.joint_position = np.zeros(JOINT_SIZE)
        self.joint_velocity = np.zeros(JOINT_SIZE)

        self.thetalist = np.zeros(JOINT_SIZE)#current state
        self.dthetalist = np.zeros(JOINT_SIZE)
        self.ddthetalist = np.zeros(JOINT_SIZE)

        self.thetalistd = np.zeros(JOINT_SIZE)
        self.dthetalistd = np.zeros(JOINT_SIZE)
        self.ddthetalistd = np.zeros(JOINT_SIZE)
        self.Ftip = np.zeros(JOINT_SIZE)
        #Integrated error
        self.eint = np.zeros(JOINT_SIZE)
    
        ##TODO, MAKE IT AS MATRIX
        self.Kp = 10
        self.Ki = 0
        self.Kd = 5
        ###################TODO: define cartesian_stiffness, cartesian_damping#############################
        # Assuming this is defined elsewhere:
        #self.cartesian_stiffness_target_array = np.array([300, 300, 300, 50, 50, 50])
        self.cartesian_stiffness_target_array = np.array([300, 300, 300, 50, 50, 50])
        self.cartesian_stiffness_target = np.diag(self.cartesian_stiffness_target_array)
        self.cartesian_damping_target_array = 2.0 * np.sqrt(self.cartesian_stiffness_target_array)
        self.cartesian_damping_target = np.diag(self.cartesian_damping_target_array)
        ###################TODO: nullspace ################################
        ###################TODO: ###########################################
        self.ee_pose = None # to hold the pinocchio pose
        self.ee_pos =np.zeros(3)
        self.ee_pos_d =None
        self.ee_ort =None
        self.ee_ort_d =None
        
        
        self.gravity = np.array([0, 0, -9.81])
        ####Module depended cosntant params
        # self.Mlist = Mlist
        # self.Glist = Glist
        # self.Slist = Slist
        # ##

        rospy.Subscriber("/ur10e/joint_states", JointState, self.ur_callback)
        rospy.Subscriber("/desired_pose", PoseStamped, self.desired_pose_callback) #TODO: receive the target pose of the interactive marker

        self.publishers = [
            rospy.Publisher("/ur10e/shoulder_pan_joint_effort_controller/command", Float64, queue_size=10),
            rospy.Publisher("/ur10e/shoulder_lift_joint_effort_controller/command", Float64, queue_size=10),
            rospy.Publisher("/ur10e/elbow_joint_effort_controller/command", Float64, queue_size=10),
            rospy.Publisher("/ur10e/wrist_1_joint_effort_controller/command", Float64, queue_size=10),
            rospy.Publisher("/ur10e/wrist_2_joint_effort_controller/command", Float64, queue_size=10),
            rospy.Publisher("/ur10e/wrist_3_joint_effort_controller/command", Float64, queue_size=10),
        ]

    def ur_callback(self, msg):
        # UR10 joint_states order adjustment
        self.joint_position = np.array([
            msg.position[2], msg.position[1], msg.position[0],
            msg.position[3], msg.position[4], msg.position[5]
        ])
        self.joint_velocity = np.array([
            msg.velocity[2], msg.velocity[1], msg.velocity[0],
            msg.velocity[3], msg.velocity[4], msg.velocity[5]
        ])
        
    def desired_pose_callback(self, msg: PoseStamped):
        # Extract translation
        position = msg.pose.position
        self.ee_pos_d = np.array([position.x, position.y, position.z])

        # Extract orientation (quaternion)
        orientation = msg.pose.orientation
        # self.ee_ort_d = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        # Construct Pinocchio quaternion from [x, y, z, w]
        self.ee_ort_d = pin.Quaternion(np.array([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]))
        #rospy.logwarn(f"target: pos = {self.ee_pos_d}, quat = {self.ee_ort_d}")


    def run(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            
            self.thetalist = self.joint_position
            #rospy.logwarn(f"thetalist: {self.thetalist}")
            self.dthetalist = self.joint_velocity
            ##Q: how the subscriber works
            pin.forwardKinematics(self.model, self.data, self.thetalist)
            pin.updateFramePlacements(self.model, self.data) 
            self.ee_pose = self.data.oMf[self.frame_id]
            self.ee_pos = self.ee_pose.translation
            self.ee_ort = pin.Quaternion(self.ee_pose.rotation)
            if(self.ee_ort_d is None or self.ee_pos_d is None):
                self.ee_ort_d = pin.Quaternion(self.ee_pose.rotation)
                self.ee_pos_d = self.ee_pose.translation
            # rospy.logwarn(f"orient: {self.ee_pose}")
            # print(self.ee_ort) (x,y,z,w)
            
            tau = self.computed_torque_ftip()

            for i in range(JOINT_SIZE):
                self.publishers[i].publish(Float64(tau[i]))

            #rospy.logwarn(f"Torques: {tau}")
            rate.sleep()

    
    
    def computed_torque_ftip(self):
            e = self.thetalistd - self.thetalist
            self.eint += e  # Integrate error

            #########################################FEEDFOWARD OF JOINT SPACE#########################################
            # Mass matrix
            M = pin.crba(self.model, self.data, self.thetalist)
            # Control acceleration (PD+I law)
            tau_ff2 = M @ (self.Kp * e + self.Kd * (self.dthetalistd - self.dthetalist) + self.Ki * self.eint)
            ###########################################DYNAMIC COMPENSTTION#########################################
            # Nonlinear effects (Coriolis + Gravity)
            tau_inv_dyn2 = pin.rnea(self.model, self.data, self.thetalist, self.dthetalist, np.zeros(JOINT_SIZE))
            # tau = tau_ff1 + tau_inv_dyn1
            #####################################Cartesian Impedance Control#########################################
            # rospy.logwarn(f"error pos: {self.ee_pos}")
            # rospy.logwarn(f"error pos: {self.ee_pos_d}")##Q:it seems like at the first callback, self.ee_pos_d is not updated yet by marker
            error_pos = self.ee_pos - self.ee_pos_d
            error_rot_base = compute_orientation_error(self.ee_ort, self.ee_ort_d, self.ee_pose) 
            error_pos = error_pos.reshape(3, 1)
            error_rot_base = error_rot_base.reshape(3, 1)
            error = np.vstack((error_pos, error_rot_base))
            
            jacobian = pin.computeFrameJacobian(self.model, self.data, self.thetalist, self.frame_id, pin.ReferenceFrame.WORLD)
            #Q:WHY?
            # 1. Compute end-effector spatial velocity
            velocity = jacobian @ self.dthetalist  # shape (6,)
            velocity = velocity.reshape((6,1))
            # 2. Compute desired Cartesian wrench
            F_ee_des = -self.cartesian_stiffness_target @ error #- self.cartesian_damping_target @ velocity  # shape (6,), the result is 6,6
            # rospy.logwarn(f"F_ee_des: {F_ee_des}")
            tau_task = jacobian.T @ F_ee_des
            # rospy.logwarn(f"tau_task: {tau_task}")
            print("error_pos:", error_pos)
            print("error_rot_base:", error_rot_base)
            print("error:", error)
            print("jacobian:", jacobian.shape)
            print("dthetalist:", self.dthetalist.shape)
            print("velocity:", velocity.shape)
            print("cartesian_stiffness_target:", self.cartesian_stiffness_target.shape)
            print("cartesian_damping_target:", self.cartesian_damping_target.shape)
            print("F_ee_des:", F_ee_des.shape)

            ######################################TODO: NullSpace###############################################################
            ######################################TODO: Tau tool###############################################################
            
            ## Q: how to make sure they are actually read before we call this function?????self.ee_pose is an SE3 object
            # rospy.logwarn(f"error pos: {error_pos}")
            #rospy.logwarn(f"error orient: {error_rot_base}")
            
            
            #tau = tau_task + tau_ff2 + tau_inv_dyn2
            tau_task = tau_task.flatten()
            tau = tau_task + tau_inv_dyn2
            # print("tau_inv_dyn2:", tau_inv_dyn2.shape) #(6,) for publishing
            
            #tau = tau_inv_dyn2
            return tau
        
def compute_orientation_error(orientation: pin.Quaternion, orientation_d: pin.Quaternion, transform: pin.SE3):
        # Ensure shortest rotation (same hemisphere)
        # if orientation_d.coeffs().dot(orientation.coeffs()) < 0.0:
        #     orientation.coeffs()[:] *= -1.0  # Flip quaternion, but readonly
        if orientation_d.coeffs().dot(orientation.coeffs()) < 0.0:
            orientation = pin.Quaternion(-orientation.coeffs())

        # Compute the difference quaternion: q_err = inv(q) * q_des
        q_error = orientation.inverse() * orientation_d
        #rospy.logwarn(f"q error: {q_error}")
        # Take the imaginary part (x, y, z) of the error quaternion
        x,y,z,w = q_error.coeffs()
        error_rot = np.array([x,y,z])

        # Transform the error to the base frame: -R * error
        error_rot_base = -transform.rotation @ error_rot
        # This works as expected, because NumPy treats a (3,) array like a column vector when used with matrix multiplication (@). 
        # So if transform.rotation is a (3, 3) matrix, then: @ error_rot is valid
        # The result is a (3,) array (still flat)

        return error_rot_base
    
if __name__ == "__main__":
    rospy.init_node("ur10e_force_control_client")
    node = ForceControlClientSubscriber()
    node.run()





