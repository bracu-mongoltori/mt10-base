import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import URDFLink

# Define the arm chain
arm_chain = Chain(name='arm', links=[
    URDFLink(
        name="Joint 1",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),
    URDFLink(
        name="Joint 2",
        origin_translation=[15.5, 0, 0],  # Length of the first arm segment
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),
    URDFLink(
        name="End Effector",
        origin_translation=[14.0, 0, 0],  # Length of the second arm segment
        origin_orientation=[0, 0, 0],
        translation=[0, 0, 0],
        rotation=None,
        joint_type='prismatic'  # Specify the joint type as 'prismatic'
    )
])

# Target position in the 2D plane (x, y)
target_position = [19.30, -10.5, 0]  # z-coordinate is fixed for the 2D plane

# Perform inverse kinematics to calculate the joint angles
initial_position = [0] * len(arm_chain.links)  # Initial joint angles
joint_angles = arm_chain.inverse_kinematics(target_position, initial_position)
print(joint_angles)
# Forward kinematics to calculate the positions of the joints
joint_positions = arm_chain.forward_kinematics(joint_angles)

# Extract x and y positions from the joint positions
x_positions = [pos[0] for pos in joint_positions]
y_positions = [pos[1] for pos in joint_positions]

print(joint_positions)

print("Computed position vector : %s, original position vector : %s" % (joint_positions[:3, 3], target_position))

# Plotting the arm
plt.plot(x_positions, y_positions, marker='o')
plt.ylim(-20, 20)
plt.axhline(0, color="black", linewidth=0.5, linestyle="--")
plt.axvline(0, color="black", linewidth=0.5, linestyle="--")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("2DOF Arm Controlled by ikpy")
# plt.legend()
plt.grid()
plt.show()
