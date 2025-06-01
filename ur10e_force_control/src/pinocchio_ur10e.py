import pinocchio as pin
from os.path import join
import numpy as np

# Load the model from a URDF
model_path = "/home/danningzhao/modern_robotics_ws/src/UR10e_force_control_gazebo/ur10e_description/urdf"
urdf_filename = "ur10e.urdf"
urdf_path = join(model_path, urdf_filename)

#model = pin.buildModelsFromUrdf(urdf_path) returns a tuple, kinematic model, collsision model and visual model
model = pin.buildModelFromUrdf(urdf_path) #only kinematic
data = model.createData()

# Set joint configuration (neutral pose)
q = pin.neutral(model)
print("neutral q",q)
for idx, name in enumerate(model.names):
    print(f"{idx}: {name}")
# Compute forward kinematics
pin.forwardKinematics(model, data, q)

# Get pose of a specific frame (e.g. 'ee_link')
frame_name = "ee_link"
frame_id = model.getFrameId(frame_name)
ee_pose = data.oMf[frame_id]  # This is an SE3 object

# Extract translation and quaternion
translation = ee_pose.translation
quaternion = pin.Quaternion(ee_pose.rotation)


print("Position:", translation)
print("Orientation (quaternion):", quaternion.coeffs())  # [x, y, z, w]

q = np.array([-1.51230768e-02,5.59536066e-01,-2.94963502e-02,-6.21821317e+00,-1.43756543e-04,-1.22066578e-05])
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data) 
# Get pose of a specific frame (e.g. 'ee_link')
frame_name = "ee_link"
frame_id = model.getFrameId(frame_name)
ee_pose = data.oMf[frame_id]  # This is an SE3 object

# Extract translation and quaternion
translation = ee_pose.translation
quaternion = pin.Quaternion(ee_pose.rotation)

print("Position:", translation)
print("Orientation (quaternion):", quaternion.coeffs())  # [x, y, z, w]

#-----------------------------------------------------------------------------------------------------------
# MODERN ROBOTICS CONVENTIONS
#-----------------------------------------------------------------------------------------------------------
# -------------------------
# Mlist: Home configuration transforms (SE3)
# -------------------------
Mlist = [model.jointPlacements[i] for i in range(1, model.njoints)]

# -------------------------
# Glist: Spatial inertias
# -------------------------
Glist = [inertia for inertia in model.inertias[1:]]  # Skip universe

# -------------------------
# Slist: Screw axes in world frame
# -------------------------
#Q: how???
# Slist = []
# for joint in model.joints[1:]:
#     motion_subspace = joint.motionSubspace()  # directly on joint
#     T = model.jointPlacements[joint.id()]
#     for j in range(joint.nv):
#         screw_local = motion_subspace.vector[j]
#         screw_world = T.act(screw_local)
#         Slist.append(screw_world)

# # -------------------------
# # Blist: Screw axes in body (local joint) frame
# # -------------------------
# Blist = []

# for i in range(1, model.njoints):
#     joint_model = model.joints[i].jointModel
#     motion_subspace = joint_model.motionSubspace()
#     for j in range(joint_model.nv):
#         screw_local = motion_subspace.vector[j]
#         Blist.append(screw_local)

# # -------------------------
# # Print sample output
# # -------------------------
# print("Number of joints:", model.njoints - 1)
# print("\nMlist[0] (homogeneous matrix):\n", Mlist[0].homogeneous)
# print("\nGlist[0] (spatial inertia):\n", Glist[0].matrix())
# print("\nSlist[0] (world frame screw):\n", Slist[0].vector)
# print("\nBlist[0] (body frame screw):\n", Blist[0].vector)

#-----------------------------------------------------------------------------------------------------------
# LAGRANGIAN FORMULATION
#-----------------------------------------------------------------------------------------------------------
# Assume model and data already created, and q, dq, ddq are given:
q = pin.neutral(model)  # joint positions
dq = np.zeros(model.nq)  # joint velocities
ddq = np.zeros(model.nq)  # joint accelerations

# 1. Compute Inertia Matrix
pin.crba(model, data, q)
M = data.M  # inertia matrix

# 2. Compute Coriolis matrix
pin.computeCoriolisMatrix(model, data, q, dq)
C = data.C  # Coriolis matrix
Cdq = C.dot(dq)  # coriolis and centrifugal vector

# 3. Compute Gravity vector
pin.computeGeneralizedGravity(model, data, q)
G = data.g

# 4. Compute inverse dynamics torques for ddq
tau = pin.rnea(model, data, q, dq, ddq)

print("Inertia matrix M:\n", M)
print("Coriolis and centrifugal vector C*dq:\n", Cdq)
print("Gravity vector G:\n", G)
print("Inverse dynamics torques tau:\n", tau)