import angle_computation as angle_math
import itertools
import pickle
import numpy as np
import copy
from xsens_data import XSENS_NAME_MAPPING

METRAPS_POINT_CLOUD = {
    "RightUpperLeg": [96],  # [1, 96, 97, 98, 99],
    "RightLowerLeg": [103],  # [4, 100, 101, 102, 103],
    "RightFoot": [78],  # [7, 78, 79, 80, 81]
    "LeftUpperLeg": [48],  # [0, 48, 49, 50, 51],
    "LeftLowerLeg": [55],  # [3, 52, 53, 54, 55],
    "LeftFoot": [30],  # [6, 30, 31, 32, 33]
    "Pelvis": [77],
    "L5": [120],  # [5, 8, 118, 119, 120, 121],
    "RightToe": [111],  # [10, 92, 93, 109, 110, 111],
    "LeftToe": [63],  # [9, 44, 45, 61, 62, 63]
    "RightUpperArm": [16],  # [16, 104, 105, 106, 107]
    "RightForeArm": [87],  # [18, 85, 86, 87, 88],
    "RightHand": [20],  # [20, 108, 112, 113, 114, 115]
    "LeftUpperArm": [15],  # [15, 56, 57, 58, 59],
    "LeftForeArm": [39],  # [17, 37, 38, 39, 40],
    "LeftHand": [19]  # [19, 49, 64, 65, 66, 67]
}

# METRAPS_POINT_CLOUD = {
#     "RightUpperLeg": [1, 96, 97, 98, 99],
#     "RightLowerLeg": [4, 100, 101, 102, 103],
#     "RightFoot": [7, 78, 79, 80, 81],
#     "LeftUpperLeg": [0, 48, 49, 50, 51],
#     "LeftLowerLeg": [3, 52, 53, 54, 55],
#     "LeftFoot": [6, 30, 31, 32, 33],
#     "Pelvis": [23, 74, 75, 76, 77],
#     "L5": [5, 8, 118, 119, 120, 121],
#     "RightToe": [10, 92, 93, 109, 110, 111],
#     "LeftToe": [9, 44, 45, 61, 62, 63]
# }

KNEE_RIGHT = {
    "name": "Knee Right",
    "function": angle_math.calculate_knee_rotation,
    "positional_xsens_params": [
        "RightUpperLeg",
        "RightLowerLeg",
        "RightFoot",
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["RightUpperLeg"],
        METRAPS_POINT_CLOUD["RightLowerLeg"],
        METRAPS_POINT_CLOUD["RightFoot"],
    ]
}

KNEE_LEFT = {
    "name": "Knee Left",
    "function": angle_math.calculate_knee_rotation,
    "positional_xsens_params": [
        "LeftUpperLeg",
        "LeftLowerLeg",
        "LeftFoot",
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["LeftUpperLeg"],
        METRAPS_POINT_CLOUD["LeftLowerLeg"],
        METRAPS_POINT_CLOUD["LeftFoot"],
    ]
}

BACK = {
    "name": "Back",
    "function": angle_math.calculate_ergo_pelvis_rotation,
    "positional_xsens_params": [
        "RightUpperLeg",
        "Pelvis",
        "T12",
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["RightUpperLeg"],
        METRAPS_POINT_CLOUD["Pelvis"],
        METRAPS_POINT_CLOUD["L5"],
    ],
    "named_metrabs_params": {
        "up_vector": [0.0, -1.0, 0.0]
    }
}

# BACK = {
#     "name": "Back",
#     "function": angle_math.calculate_ergo_pelvis_rotation,
#     "positional_xsens_params": [
#         "RightShoulder",
#         "T8",
#         "Neck",
#     ],
#     "positional_metrabs_params": [
#         [82],
#         [68],
#         [11, 69, 70],
#     ],
#     "named_metrabs_params": {
#         "up_vector": [0.0, -1.0, 0.0]
#     }
# }

ELLBOW_RIGHT = {
    "name": "Ellbow Right",
    "function": angle_math.calculate_knee_rotation,
    "positional_xsens_params": [
        "RightUpperArm",
        "RightForeArm",
        "RightHand",
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["RightUpperArm"],
        METRAPS_POINT_CLOUD["RightForeArm"],
        METRAPS_POINT_CLOUD["RightHand"],
    ]
}

ELLBOW_LEFT = {
    "name": "Ellbow Left",
    "function": angle_math.calculate_knee_rotation,
    "positional_xsens_params": [
        "LeftUpperArm",
        "LeftForeArm",
        "LeftHand",
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["LeftUpperArm"],
        METRAPS_POINT_CLOUD["LeftForeArm"],
        METRAPS_POINT_CLOUD["LeftHand"],
    ]
}

ANKLE_RIGHT = {
    "name": "Ankle Right",
    "function": angle_math.calculate_ankle_rotation,
    "positional_xsens_params": [
        "RightUpperLeg",
        "RightLowerLeg",
        "RightFoot",
        "RightToe"
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["RightUpperLeg"],
        METRAPS_POINT_CLOUD["RightLowerLeg"],
        METRAPS_POINT_CLOUD["RightFoot"],
        METRAPS_POINT_CLOUD["RightToe"]
    ]
}

ANKLE_Left = {
    "name": "Ankle Left",
    "function": angle_math.calculate_ankle_rotation,
    "positional_xsens_params": [
        "RightUpperLeg",
        "RightLowerLeg",
        "RightFoot",
        "RightToe",
    ],
    "positional_metrabs_params": [
        METRAPS_POINT_CLOUD["LeftUpperLeg"],
        METRAPS_POINT_CLOUD["LeftLowerLeg"],
        METRAPS_POINT_CLOUD["LeftFoot"],
        METRAPS_POINT_CLOUD["LeftToe"]
    ]
}

DEFINITIONS = [KNEE_RIGHT, KNEE_LEFT, BACK, ANKLE_RIGHT, ANKLE_Left, ELLBOW_LEFT, ELLBOW_RIGHT]


class EvaluationSummary:
    def __init__(self, name):
        self._data = {}
        self._name = name

    def _add_angle_diff(self, tag, point_indices, angle_diff):
        point_str = str(point_indices)
        if point_str not in self._data:
            self._data[point_str] = {}
        self._data[point_str][tag] = angle_diff

    def add_avg_angle_diff(self, point_indices, angle_diff):
        self._add_angle_diff("avg", point_indices, angle_diff)

    def add_median_angle_diff(self, point_indices, angle_diff):
        self._add_angle_diff("median", point_indices, angle_diff)

    def add_max_angle_diff(self, point_indices, angle_diff):
        self._add_angle_diff("max", point_indices, angle_diff)

    def add_min_angle_diff(self, point_indices, angle_diff):
        self._add_angle_diff("min", point_indices, angle_diff)

    def print_top_flextion_angles(self, count=10, tag="avg"):
        """Print top `count` angles. Choose between the tag avg, median, min, max"""
        possible_tags = ["avg", "median", "min", "max"]
        if tag not in possible_tags:
            raise ValueError(f"tag must be in {possible_tags}")

        data_list = [{"position_name": k, **v} for k, v in self._data.items()]

        print(f"--- Top {count} - {self._name} - sorted {tag} ---")
        sorted_data = sorted(data_list, key=lambda x: x[tag][2])
        iteration_count = min(len(sorted_data), count)
        for i in range(iteration_count):
            data = sorted_data[i]
            positions = data["position_name"]
            tag_strings = [f" {tag}: {data[tag]}".ljust(28) for tag in possible_tags if tag in self._data[positions]]
            data_str = "".join(tag_strings)
            print(positions.ljust(15), data_str)


def evaluate_definition(definition: dict) -> EvaluationSummary:
    angle_computation_function = definition["function"]
    xsens_indices = definition["positional_xsens_params"]
    metrabs_indices_combination = list(itertools.product(*definition["positional_metrabs_params"]))
    metrabs_named_args = definition.get("named_metrabs_params", {})

    snapshots = pickle.load(open("./metrabs_data.pickle", "rb"))

    # store all angle diff for each combination
    angle_diffs = [[] for _ in range(len(metrabs_indices_combination))]

    for data_index in range(len(snapshots)):
        metrabs_pos = snapshots[data_index]["metrabs_pos"]
        xsens_pos = snapshots[data_index]["awinda_pos"]
        xsens_params = [xsens_pos[XSENS_NAME_MAPPING[i]] for i in xsens_indices]

        xsens_angle = angle_computation_function(*xsens_params)
        xsens_angle = xsens_angle[0] * (180 / np.pi)

        metrabs_params_combination = [[metrabs_pos[i] for i in p] for p in metrabs_indices_combination]
        for i, metrabs_params in enumerate(metrabs_params_combination):
            metrabs_angle = angle_computation_function(*metrabs_params, **metrabs_named_args)
            metrabs_angle = metrabs_angle[0] * (180 / np.pi)
            angle_diffs[i].append(np.absolute(metrabs_angle - xsens_angle))

    summary = EvaluationSummary(definition["name"])
    for i, point_combination in enumerate(metrabs_indices_combination):
        avg_angles = np.average(angle_diffs[i], axis=0)
        median_angles = np.median(angle_diffs[i], axis=0)
        min_angles = np.min(angle_diffs[i], axis=0)
        max_angles = np.max(angle_diffs[i], axis=0)

        summary.add_avg_angle_diff(point_combination, avg_angles)
        summary.add_median_angle_diff(point_combination, median_angles)
        summary.add_min_angle_diff(point_combination, min_angles)
        summary.add_max_angle_diff(point_combination, max_angles)

    return summary


def main():
    np.set_printoptions(suppress=True)
    np.set_printoptions(precision=3)
    evaluations = []
    for definition in DEFINITIONS:
        evaluation = evaluate_definition(definition)
        evaluation.print_top_flextion_angles(10, "avg")
        evaluation.print_top_flextion_angles(10, "median")
        evaluation.print_top_flextion_angles(10, "max")
        print()
        evaluations.append(evaluation)


if __name__ == "__main__":
    main()
