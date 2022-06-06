import numpy as np
from math import *
import csv

"""CSV File Reader
"""


def DH_load(file_path="../Model/DH.csv"):
    _DH = []
    # Load param from the file
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for i, row in enumerate(reader):
            if i != 0:
                _DH.append([eval(col) for col in row])

    DH = np.asarray(_DH)
    DHJoint = DH[0:len(DH) - 1]
    ToEndEffector = DH[-1]

    return DH, DHJoint, ToEndEffector


def joint_limit_load(file_path="../Model/JointLimit.csv"):
    pos_limit = []
    vel_limit = []

    with open(file_path, 'r') as file:
        reader = list(csv.reader(file))
        cnt = 0
        while cnt < len(reader):
            row = reader[cnt]

            # Position Limit
            flag = [string for string in row if "position" in string.lower()]
            if flag:
                row1, row2 = reader[cnt + 1], reader[cnt + 2]
                pos_limit.append([eval(col) for col in row1])
                pos_limit.append([eval(col) for col in row2])
                flag = []
                cnt += 2
                continue

            # Velocity Limit
            flag = [string for string in row if "velocity" in string.lower()]
            if flag:
                row1, row2 = reader[cnt + 1], reader[cnt + 2]
                vel_limit.append([eval(col) for col in row1])
                vel_limit.append([eval(col) for col in row2])
                flag = []
                cnt += 2
                continue

            cnt += 1

    pos_limit = np.asarray(pos_limit)
    vel_limit = np.asarray(vel_limit)

    return pos_limit, vel_limit
