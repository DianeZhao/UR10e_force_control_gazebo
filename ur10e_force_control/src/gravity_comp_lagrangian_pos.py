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
# m01 = np.array([
#     [1, 0, 0, 0],
#     [0, 1, 0, 0],
#     [0, 0, 1, 0.1810],
#     [0, 0, 0, 1]
# ])

# m12 = np.array([
#     [0, 0, 1, 0.3065],
#     [0, 1, 0, 0.1760],
#     [-1, 0, 0, 0],
#     [0, 0, 0, 1]
# ])

# m23 = np.array([
#     [1, 0, 0, 0],
#     [0, 1, 0, -0.1370],
#     [0, 0, 1, 0.5920],
#     [0, 0, 0, 1]
# ])

# m34 = np.array([
#     [0, 0, 1, 0],
#     [0, 1, 0, 0.135],
#     [-1, 0, 0, 0.2855],
#     [0, 0, 0, 1]
# ])

# m45 = np.array([
#     [1, 0, 0, 0],
#     [0, 1, 0, 0],
#     [0, 0, 1, 0.12],
#     [0, 0, 0, 1]
# ])

# m56 = np.array([
#     [1, 0, 0, 0],
#     [0, 1, 0, 0.092],
#     [0, 0, 1, 0],
#     [0, 0, 0, 1]
# ])

# m67 = np.array([
#     [1, 0, 0, 0],
#     [0, 0, 1, 0.025],
#     [0, -1, 0, 0],
#     [0, 0, 0, 1]
# ])

# G1 = np.identity(6)
# G1[0, 0] *= 0.03147431257693659
# G1[1, 1] *= 0.03147431257693659
# G1[2, 2] *= 0.021875625
# G1[3, 3] *= 7.778
# G1[4, 4] *= 7.778
# G1[5, 5] *= 7.778

# G2 = np.identity(6)
# G2[0, 0] *= 0.4230737407704359
# G2[1, 1] *= 0.4230737407704359
# G2[2, 2] *= 0.036365625
# G2[3, 3] *= 12.93
# G2[4, 4] *= 12.93
# G2[5, 5] *= 12.93

# G3 = np.identity(6)
# G3[0, 0] *= 0.11059036576383598
# G3[1, 1] *= 0.11059036576383598
# G3[2, 2] *= 0.010884375
# G3[3, 3] *= 3.87
# G3[4, 4] *= 3.87
# G3[5, 5] *= 3.87

# G4 = np.identity(6)
# G4[0, 0] *= 0.005108247956699999
# G4[1, 1] *= 0.005108247956699999
# G4[2, 2] *= 0.0055125
# G4[3, 3] *= 1.96
# G4[4, 4] *= 1.96
# G4[5, 5] *= 1.96

# G5 = np.copy(G4)

# G6 = np.identity(6)
# G6[0, 0] *= 0.00014434577559500002
# G6[1, 1] *= 0.00014434577559500002
# G6[2, 2] *= 0.000204525
# G6[3, 3] *= 0.202
# G6[4, 4] *= 0.202
# G6[5, 5] *= 0.202

# Mlist = [m01, m12, m23, m34, m45, m56, m67]#4x24
# Glist = [G1, G2, G3, G4, G5, G6]#6x36

# Slist = np.array([
#     [0,    0,    0,    0,     0,     0],
#     [0,    1,    1,    1,     0,     1],
#     [1,    0,    0,    0,    -1,     0],
#     [0, -0.1810, -0.1810, -0.1810, -0.1760, -0.0610],
#     [0,    0,    0,    0,   1.1840,    0],
#     [0,    0, 0.6130, 1.1840,   0,   1.1840]
# ])



class ForceControlClientSubscriber:
    def __init__(self):
        
        #model = pin.buildModelsFromUrdf(urdf_path) returns a tuple, kinematic model, collsision model and visual model
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
        ###################TODO: define cartesian_stiffness, cartesian_damping################################
        ###################TODO: nullspace ################################
        ###################TODO: ###########################################
        self.ee_pos =np.zeros(3)
        self.ee_pos_d =np.array([1.16844, 0.291643,1.57469])
        # self.ee_ort =np.zeros(JOINT_SIZE)
        # self.ee_ort_d =np.zeros(JOINT_SIZE)
        self.cartesian_stiffness_target_array = np.array([100, 100, 100, 10, 10, 10])
        self.cartesian_stiffness_target = np.diag(self.cartesian_stiffness_target_array)
        self.cartesian_damping_target_array = 2.0 * np.sqrt(self.cartesian_stiffness_target_array)
        self.cartesian_damping_target = np.diag(self.cartesian_damping_target_array)
        
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
        self.ee_ort_d = np.array([orientation.w, orientation.x, orientation.y, orientation.z])

    def run(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            self.thetalist = self.joint_position
            self.dthetalist = self.joint_velocity
            
            pin.forwardKinematics(self.model, self.data, self.thetalist)
            pin.updateFramePlacements(self.model, self.data)
            self.jacobian = pin.computeFrameJacobian(self.model, self.data, self.thetalist, self.frame_id, pin.ReferenceFrame.WORLD)
            self.ee_pose = self.data.oMf[self.frame_id]
            self.ee_pos = self.ee_pose.translation
            # self.ee_ort = pin.Quaternion(self.ee_pose.rotation)
            print(self.ee_pos)
            # print(self.ee_ort)
            #  R =
            # -0.000568947     0.992678    -0.120792
            #         1  0.000565077 -6.62952e-05
            # 2.44675e-06    -0.120792    -0.992678
            # p =  1.16844 0.291643  1.57469

            # (x,y,z,w) =    0.70561   0.706012  -0.042796 -0.0427734
            tau = self.computed_torque_ftip()
            
            for i in range(JOINT_SIZE):
                self.publishers[i].publish(Float64(tau[i]))

            #rospy.logwarn(f"Torques: {tau}")
            rate.sleep()

    # def computed_torque_ftip(self):
    #     e = self.thetalistd - self.thetalist
    #     self.eint += e  # Integrate error

    #     # M = mr.MassMatrix(self.thetalist, self.Mlist, self.Glist, self.Slist)
    #     # tau_ff1 = M @ (self.Kp * e + self.Ki * self.eint + self.Kd * (self.dthetalistd - self.dthetalist))

    #     # tau_inv_dyn1 = mr.InverseDynamics(self.thetalist, self.dthetalist, self.ddthetalistd,
    #     #                                  self.gravity, self.Ftip, self.Mlist, self.Glist, self.Slist)

    #     # return tau_ff + tau_inv_dyn
        
    #     # Mass matrix
    #     M = pin.crba(self.model, self.data, self.thetalist)
    #     # Control acceleration (PD+I law)
    #     tau_ff2 = M @ (self.Kp * e + self.Kd * (self.dthetalistd - self.dthetalist) + self.Ki * self.eint)
    #     # Nonlinear effects (Coriolis + Gravity)
    #     error = np.zeros(6)
    #     error[:3] = self.ee_pos - self.ee_pos_d
    #     print("error",error)
    #     v_ee = pin.getFrameVelocity(self.model, self.data, self.frame_id, pin.ReferenceFrame.WORLD).linear
    #     F_lin = -self.cartesian_stiffness_target[:3, :3] @ error[:3] - self.cartesian_damping_target[:3, :3] @ v_ee
    #     # Construct full 6D force vector
    #     F_ee_des = np.zeros(6)
    #     F_ee_des[:3] = F_lin  # Force part
    #     # F_ee_des[3:] = 0      # No torque control (optional, explicit)
    #     print("F", F_ee_des)
    #     tau_ff3 = self.jacobian.T @ F_ee_des
    #     print("tau_ff3", tau_ff3)
    #     tau_inv_dyn2 = pin.rnea(self.model, self.data, self.thetalist, self.dthetalist, np.zeros(JOINT_SIZE))
    #     # tau = tau_ff1 + tau_inv_dyn1
    #     tau = tau_ff3 + tau_inv_dyn2
    #     return tau
    def computed_torque_ftip(self):
        # Joint space error (for diagnostics or fallback control)
        e = self.thetalistd - self.thetalist
        self.eint += e

        # Cartesian pose error
        error = np.zeros(6)
        error[:3] = self.ee_pos - self.ee_pos_d

        # Clamp Cartesian error to avoid huge forces
        max_cartesian_error = 0.05  # [m]
        error[:3] = np.clip(error[:3], -max_cartesian_error, max_cartesian_error)

        # End-effector linear velocity
        v_ee = pin.getFrameVelocity(self.model, self.data, self.frame_id, pin.ReferenceFrame.WORLD).linear

        # Cartesian impedance force (linear only)
        F_lin = -self.cartesian_stiffness_target[:3, :3] @ error[:3] \
                - self.cartesian_damping_target[:3, :3] @ v_ee

        # Clamp Cartesian force
        F_lin = np.clip(F_lin, -50, 50)

        # ✅ Only use linear velocity Jacobian
        Jv = self.jacobian[:3, :]  # shape: (3 × n)
        tau_ff3 = Jv.T @ F_lin     # shape: (n × 3) @ (3,) → (n,)

        # Add gravity + coriolis compensation
        tau_inv_dyn = pin.rnea(self.model, self.data, self.thetalist, self.dthetalist, np.zeros(JOINT_SIZE))

        # Final torque
        tau = tau_ff3 + tau_inv_dyn
        return tau


if __name__ == "__main__":
    rospy.init_node("ur10e_force_control_client")
    node = ForceControlClientSubscriber()
    node.run()





