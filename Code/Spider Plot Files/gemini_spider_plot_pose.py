import numpy as np
from math import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# --- Helper function: axis-angle rotation matrix ---
def axis_angle_rotation_matrix(axis, angle):
    """
    Computes the 3D rotation matrix for rotation around an axis by an angle.
    Equivalent to MATLAB's axis_angle_rotation_matrix.
    """
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    C = 1 - c

    R = np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ]
    )
    return R


# --- Helper function: rotate a vector around an axis by an angle ---
def rotate_vector(v, axis, angle):
    """
    Rotates vector v around axis by angle.
    Equivalent to MATLAB's rotate_vector: v_rot = (R * v')'.
    In NumPy, v_rot = R @ v.
    """
    R = axis_angle_rotation_matrix(axis, angle)
    # The original MATLAB code uses (R * v')', where v is a row vector.
    # In NumPy, this is R @ v.T which returns a column vector,
    # so we return v_rot.T to match the expected row vector output.
    v_rot = R @ v.T
    return v_rot.T


def forward_leg_kinematics2(base_pos, base_angle, joint_angles, segment_lengths):
    """
    forward_leg_kinematics2 - Compute the forward kinematics for a single spider leg.
    Equivalent to MATLAB's forward_leg_kinematics2 function.

    Input:
        base_pos: [x,y,z] numpy array position of leg base on body
        base_angle: angle around body ellipse where leg base is located (radians)
        joint_angles: [theta1, theta2, theta3] joint angles for the leg in radians
        segment_lengths: [coxa, femur, tibia] lengths of leg segments

    Output:
        j1, j2, j3, j4: numpy arrays [x,y,z] of the joint positions.
    """
    # Unpack joint angles (theta1: Coxa yaw, theta2: Femur pitch, theta3: Tibia pitch)
    theta1, theta2, theta3 = joint_angles

    # Unpack segment lengths
    L1, L2, L3 = segment_lengths  # Coxa, Femur, Tibia lengths

    # Joint 1: leg base on body
    j1 = np.array(base_pos)  # starting point

    # --- Compute Coxa direction with elevation ---
    coxa_elevation = np.radians(30)  # fixed 30 degree upward pitch for coxa

    # Horizontal direction of coxa in XY plane based on base_angle + theta1
    coxa_horiz_dir = np.array(
        [np.cos(base_angle + theta1), np.sin(base_angle + theta1), 0]
    )

    # Rotation axis for pitch up: perpendicular to coxa horizontal direction in XY plane
    # Equivalent to MATLAB's cross(coxa_horiz_dir, [0 0 1])
    rot_axis = np.cross(coxa_horiz_dir, [0, 0, 1])

    # Rotation matrix around rot_axis by coxa_elevation
    R = axis_angle_rotation_matrix(rot_axis, coxa_elevation)

    # Rotate horizontal coxa direction upward
    # Equivalent to MATLAB's (R * coxa_horiz_dir')'
    coxa_dir = (R @ coxa_horiz_dir.T).T

    # Joint 2 position: end of coxa segment
    j2 = j1 + L1 * coxa_dir

    # --- Femur rotation ---
    # Define femur rotation axis (perpendicular to coxa_dir and vertical axis)
    femur_rot_axis = np.cross(coxa_dir, [0, 0, 1])
    # The original MATLAB code does implicit normalization in axis_angle_rotation_matrix,
    # but explicit normalization is safer here.
    femur_rot_axis = femur_rot_axis / np.linalg.norm(femur_rot_axis)

    # Femur direction vector starts aligned with coxa_dir
    femur_dir = rotate_vector(coxa_dir, femur_rot_axis, theta2)

    # Joint 3 position: end of femur segment
    j3 = j2 + L2 * femur_dir

    # --- Tibia rotation ---
    # Tibia pitch is relative to femur direction
    tibia_rot_axis = np.cross(femur_dir, [0, 0, 1])
    tibia_rot_axis = tibia_rot_axis / np.linalg.norm(tibia_rot_axis)

    # Tibia direction vector
    tibia_dir = rotate_vector(femur_dir, tibia_rot_axis, theta3)

    # Joint 4 position: end of tibia segment (foot)
    j4 = j3 + L3 * tibia_dir

    return j1, j2, j3, j4


def plot_spider_pose(angles):
    """
    plot_spider_pose - Plot a static 3D spider pose based on joint angles.
    Equivalent to MATLAB's plot_spider_pose function.

    Input:
        angles: 1x24 numpy array of joint angles in radians
            [theta1_1, theta2_1, theta3_1, ..., theta1_8, theta2_8, theta3_8]
    """
    # Parameters
    n_legs = 8
    segment_lengths = np.array([1.2, 0.7, 1.0])  # [Coxa, Femur, Tibia]
    a = 1.5
    b = 1.0  # Ellipse axes for body (oval shape)

    # Base angles (L1 front-left to L4 rear-left, R4 rear-right to R1 front-right)
    left_leg_angles = np.radians([45, 75, 105, 135])
    right_leg_angles = np.radians([-135, -105, -75, -45])
    base_angles = np.concatenate([left_leg_angles, right_leg_angles])

    # Leg labels
    leg_labels = ["L1", "L2", "L3", "L4", "R4", "R3", "R2", "R1"]

    # Validate input
    if angles.size != n_legs * 3:
        raise ValueError(
            "Input angles must be a 1x24 vector (3 angles per leg for 8 legs)."
        )

    # Setup figure
    fig = plt.figure(1)
    plt.clf()
    ax = fig.add_subplot(111, projection="3d")

    # Set background color (Matplotlib equivalent of set(gcf, 'Color', 'w'))
    fig.set_facecolor("white")

    # Axis limits and labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([-4, 4])
    ax.set_ylim([-4, 4])
    ax.set_zlim([-2, 2])

    # View angle (Matplotlib equivalent of view(45, 45))
    ax.view_init(elev=45, azim=45)

    # Equivalent to axis equal and grid on
    ax.set_box_aspect([1, 1, 1])  # Makes axes equal visually
    ax.grid(True)

    # Plot body (oval shape)
    t = np.linspace(0, 2 * np.pi, 100)
    body_x = a * np.cos(t)
    body_y = b * np.sin(t)
    body_z = np.zeros_like(t)
    ax.plot(
        body_x, body_y, body_z, color="k", linestyle="-", linewidth=3
    )  # 'k-' in MATLAB is black line

    # Head marker (front of spider at +X)
    ax.plot(
        [a + 0.2], [0], [0], marker="^", color="r", markersize=10, markerfacecolor="r"
    )

    # Print joint angles for all legs
    print("--- Spider Pose ---")

    # Loop over legs
    for i in range(n_legs):
        # Indices for this leg's angles
        idx = i * 3
        theta1 = angles[idx]
        theta2 = angles[idx + 1]
        theta3 = angles[idx + 2]

        print(
            f"Leg {leg_labels[i]}: theta1 = {theta1:.3f} rad, theta2 = {theta2:.3f} rad, theta3 = {theta3:.3f} rad"
        )

        # Compute leg base position on body ellipse
        angle = base_angles[i]
        x_base = a * np.cos(angle)
        y_base = b * np.sin(angle)
        base_pos = np.array([x_base, y_base, 0])

        # Compute FK for this leg
        j1, j2, j3, j4 = forward_leg_kinematics2(
            base_pos, angle, np.array([theta1, theta2, theta3]), segment_lengths
        )

        # Plot leg segments
        # Base (j1 to j2) - Coxa
        ax.plot([j1[0], j2[0]], [j1[1], j2[1]], [j1[2], j2[2]], color="k", linewidth=2)
        # Coxa-Femur (j2 to j3) - Femur
        ax.plot([j2[0], j3[0]], [j2[1], j3[1]], [j2[2], j3[2]], color="b", linewidth=2)
        # Femur-Tibia (j3 to j4) - Tibia
        ax.plot([j3[0], j4[0]], [j3[1], j4[1]], [j3[2], j4[2]], color="r", linewidth=2)
        # Foot marker (j4)
        ax.plot(
            [j4[0]],
            [j4[1]],
            [j4[2]],
            marker="o",
            color="r",
            markersize=5,
            markerfacecolor="r",
        )

        # Label leg
        offset = 0.2
        label_pos = base_pos + offset * np.array([np.cos(angle), np.sin(angle), 0])
        # text equivalent in Matplotlib
        ax.text(
            label_pos[0],
            label_pos[1],
            label_pos[2] + 0.05,
            leg_labels[i],
            fontsize=12,
            fontweight="bold",
        )

    plt.show()

    # Example Usage:


# Note: You need to define the functions above in the same scope or import them.
# Example for a 'neutral' stance (all pitch angles at 0, yaw angles centered)
angles = np.array(
    [
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
        0,
        -pi / 8,
        -pi / 8,
    ]
)
# Plot the pose
plot_spider_pose(angles)
