kp_joint = (
    6.0, 6.0,
    # 30., 30., 20., 20., 12., 20., 12.,
    # 25., 25., 15., 15., 8., 15., 8.,
    40., 50., 20., 10., 10., 20., 10.,
    40., 50., 20., 10., 10., 20., 10.,
    30.0,
    10.0, 10.0, 10.0, 10.0,  10.0,  10.0,
    10.0, 10.0, 10.0, 10.0,  10.0,  10.0
    )

kd_joint = (
    1.5, 1.5,
    # 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    # 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    3.0, 3.0, 2.0, 2.5, 0.7, 1.8, 0.7,
    3.0, 3.0, 2.0, 2.5, 0.7, 1.8, 0.7,
    15.0,
    5.0,  5.0,  5.0,  5.0,  0.5,  0.5,
    5.0,  5.0,  5.0,  5.0,  0.5,  0.5
    )

LEFT_ARM_NAMES = (
    "Left_Shoulder_Pitch", "Left_Shoulder_Roll",
    "Left_Elbow_Pitch", "Left_Elbow_Yaw",
    "Left_Wrist_Pitch", "Left_Wrist_Yaw", "Left_Hand_Roll"
    )

RIGHT_ARM_NAMES = (
    "Right_Shoulder_Pitch", "Right_Shoulder_Roll",
    "Right_Elbow_Pitch", "Right_Elbow_Yaw",
    "Right_Wrist_Pitch", "Right_Wrist_Yaw", "Right_Hand_Roll"
    )

ARM_NAMES = LEFT_ARM_NAMES + RIGHT_ARM_NAMES

JOINT_NAMES = ("Head_Yaw", "Head_Pitch",
    "Left_Shoulder_Pitch", "Left_Shoulder_Roll",
    "Left_Elbow_Pitch", "Left_Elbow_Yaw",
    "Left_Wrist_Pitch", "Left_Wrist_Yaw", "Left_Hand_Roll",
    "Right_Shoulder_Pitch", "Right_Shoulder_Roll",
    "Right_Elbow_Pitch", "Right_Elbow_Yaw",
    "Right_Wrist_Pitch", "Right_Wrist_Yaw", "Right_Hand_Roll",
    "Waist", "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw",
    "Left_Knee_Pitch", "Crank_Up_Left", "Crank_Down_Left",
    "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw",
    "Right_Knee_Pitch", "Crank_Up_Right", "Crank_Down_Right"
)

JOINT_PARAMETERS = {name: {"idx": idx, "kp": kp_joint[idx], "kd": kd_joint[idx]} for idx, name in enumerate(JOINT_NAMES)}
PREP_POSITIONS = (0.0, 0.0, 0.45, -1.05, 0.0, -1.5, 0.0, 0.0, 0.0,
                  0.45, -1.05, 0.0, -1.5, 0.0, 0.0, 0.0,
                  0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0)
TOTAL_DOF = len(JOINT_PARAMETERS)

if __name__ == "__main__": #DEBUGGING PURPOSES
    for name, params in JOINT_PARAMETERS.items():
        print(f"{name}: idx={params['idx']} prep={PREP_POSITIONS[params['idx']]} kp={params['kp']} kd={params['kd']}")

# DAMPING_POSITIONS = ()


#   parallel motor 0: 0.00171661376
#   parallel motor 1: 0.018119812
#   parallel motor 2: 0.46368351
#   parallel motor 3: -1.0841771
#   parallel motor 4: 0.0211718920
#   parallel motor 5: -1.46124207
#   parallel motor 6: -0.00095367435
#   parallel motor 7: 0.01506828
#   parallel motor 8: -0.0062942
#   parallel motor 9: 0.46253910
#   parallel motor 10: 1.0887547
#   parallel motor 11: 0.01773861
#   parallel motor 12: 1.4539940
#   parallel motor 13: 0.0009536743
#   parallel motor 14: -0.0101091
#   parallel motor 15: 0.005912780
#   parallel motor 16: 0.0009536888305
#   parallel motor 17: -0.096704050
#   parallel motor 18: -0.00438696
#   parallel motor 19: -0.005149920
#   parallel motor 20: 0.20008392
#   parallel motor 21: 0.105477988
#   parallel motor 22: 0.106622412
#   parallel motor 23: -0.1005188
#   parallel motor 24: 0.005149920
#   parallel motor 25: -0.005912871
#   parallel motor 26: 0.213435
#   parallel motor 27: 0.0997558
#   parallel motor 28: 0.0959410
