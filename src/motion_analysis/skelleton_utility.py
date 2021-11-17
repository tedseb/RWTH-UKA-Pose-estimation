#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the WorkerHandler Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
Based on https://github.com/Basti110/skeleton-normalization/blob/main/convert.py
"""

from typing import Set
import copy
import numpy as np
from backend.msg import Person, Bodypart

already_added_nodes : Set[int] = {} #For Debug

#Build order of the current TrainerAi skeleton
BODY_BUILD_ORDER = [(39, 8), (39, 27), (39, 28), (28, 12), (12, 9), (39, 41), (41, 1), (27, 2), (28, 5), (2, 3), (3, 4),
(5, 6), (6, 7), (27, 10), (10, 11), (11, 23), (11, 22), (28, 13), (13, 14), (14, 19), (14, 20), (1, 37),
(37, 43), (43, 38), (43, 17), (43, 18), (17, 42), (42, 0), (0, 15), (0, 16)]

CENTRAL_JOINT_IDX = 4 # Choose this to be something like the pelvis 

def resize_len_vec1_to_vec2(vec1, vec2):
    length_vec1 = np.linalg.norm(vec1)
    length_vec2 = np.linalg.norm(vec2)
    return (length_vec2 / length_vec1) * vec1

normal_skeleton = Person()
joint1_p2 : Bodypart = person2.bodyParts[body_joints[0]]
joint2_p2 : Bodypart = person2.bodyParts[body_joints[1]]
point1_p2 = np.array([joint1_p2.point.x, joint1_p2.point.y, joint1_p2.point.z])
point2_p2 = np.array([joint2_p2.point.x, joint2_p2.point.y, joint2_p2.point.z])
vec_joints_person2 = np.array(point2_p2 - point1_p2)



def normalize_skelleton_size(person: Person) -> Person:
    '''Stretch all vectors of person to the same length as the vectors in normal_skeleton

    BODY_BUILD_ORDER: The order to build a completly new skeleton with the old vectors. The first Index in the first tuple
    is the start point. It is the only point with the same coordinates as before. For the new skeleton, the algorithm
    takes the vector direction of person and the vecttor length of person2.
    '''
    resized_person = Person()
    for _ in range(len(person.bodyParts)):
        resized_person.bodyParts.append(Bodypart())

    start_bodypart = BODY_BUILD_ORDER[0][0]
    resized_person.bodyParts[start_bodypart] = copy.deepcopy(person.bodyParts[start_bodypart])

    for body_joints in BODY_BUILD_ORDER:
        joint1_p1 : Bodypart = person.bodyParts[body_joints[0]]
        joint2_p1 : Bodypart = person.bodyParts[body_joints[1]]
        

        point1_p1 = np.array([joint1_p1.point.x, joint1_p1.point.y, joint1_p1.point.z])
        point2_p1 = np.array([joint2_p1.point.x, joint2_p1.point.y, joint2_p1.point.z])
        vec_joints_person1 = np.array(point2_p1 - point1_p1)

        new_vec_p1 = resize_len_vec1_to_vec2(vec_joints_person1, vec_joints_person2)

        resized_person_bodypart1 : Bodypart = resized_person.bodyParts[body_joints[0]]
        new_bodypart = Bodypart()
        new_bodypart.point.x = resized_person_bodypart1.point.x + new_vec_p1[0]
        new_bodypart.point.y = resized_person_bodypart1.point.y + new_vec_p1[1]
        new_bodypart.point.z = resized_person_bodypart1.point.z + new_vec_p1[2]

        resized_person.bodyParts[body_joints[1]] = new_bodypart

    return resized_person

def normalize_skelleton_position_and_orientation(person: Person) -> Person:
    normalized_person = Person()
    central_joint = person.bodyParts[CENTRAL_JOINT_IDX]

    # Move skelleton to 0/0/0 by substracting coordinates of central joint from all joints
    for b in range(len(person.bodyParts)):
        new_bodypart = Bodypart()
        new_bodypart.point.x = b.point.x - central_joint[0]
        new_bodypart.point.y = b.point.y - central_joint[1]
        new_bodypart.point.z = b.point.z - central_joint[2]
        normalized_person.bodyParts.append(new_bodypart)

    # Turn skelleton around z-axis 
    
    # Find out alpha of original skelleton
    
    # Turn around z axis with formula from: https://www.biancahoegel.de/mathe/matrix/drehmatrix.html



if __name__ == '__main__':
    '''Small example test with known values'''
    person1 = Person()
    person1.bodyParts.append(Bodypart())

    bodypart1 = Bodypart()
    bodypart2 = Bodypart()
    bodypart3 = Bodypart()
    bodypart4 = Bodypart()
    bodypart1.point.x, bodypart1.point.y, bodypart1.point.z = 0, 0, 0
    bodypart2.point.x, bodypart2.point.y, bodypart2.point.z = 0, 1, 0
    bodypart3.point.x, bodypart3.point.y, bodypart3.point.z = -1, 1, 0
    bodypart4.point.x, bodypart4.point.y, bodypart4.point.z = -1, 1, 1
    person1.bodyParts.append(copy.deepcopy(bodypart1))
    person1.bodyParts.append(copy.deepcopy(bodypart2))
    person1.bodyParts.append(copy.deepcopy(bodypart3))
    person1.bodyParts.append(copy.deepcopy(bodypart4))

    person2 = Person()
    person2.bodyParts.append(Bodypart())
    bodypart1.point.x, bodypart1.point.y, bodypart1.point.z = 0, 0, 0
    bodypart2.point.x, bodypart2.point.y, bodypart2.point.z = 2, 0, 0
    bodypart3.point.x, bodypart3.point.y, bodypart3.point.z = 2, 0.5, 0
    bodypart4.point.x, bodypart4.point.y, bodypart4.point.z = 6, 0.5, 3
    person2.bodyParts.append(copy.deepcopy(bodypart1))
    person2.bodyParts.append(copy.deepcopy(bodypart2))
    person2.bodyParts.append(copy.deepcopy(bodypart3))
    person2.bodyParts.append(copy.deepcopy(bodypart4))

    # small test build order
    body_order = [(1, 2), (2, 3), (3, 4)]
    result_person = normalize_skelleton(person1, person2, body_order)
    print("### Body parts resized (Test)###")
    print("Point1, expect: 0, 0, 0")
    print(result_person.bodyParts[1].point)
    print("Point2, expect: 0, 2, 0")
    print(result_person.bodyParts[2].point)
    print("Point3, expect: -0.5, 2, 0")
    print(result_person.bodyParts[3].point)
    print("Point4, expect: -0.5, 2, 5")
    print(result_person.bodyParts[4].point)

    #Node Test. Checks Joint Coverage.
    already_added_nodes : Set[int] = set({}) #For Debug
    already_added_nodes.add(BODY_BUILD_ORDER[0][0])
    for joint in BODY_BUILD_ORDER:
        if joint[0] not in already_added_nodes:
            print(f"ERROR: Index {joint[0]} (first index) was not added in the Set before")
        if joint[1] in already_added_nodes:
            print(f"ERROR: Index {joint[0]} (second index) was added in the Set before")
        already_added_nodes.add(joint[1])
    print(f"Total number of joints: {len(already_added_nodes)}")
    print(already_added_nodes)