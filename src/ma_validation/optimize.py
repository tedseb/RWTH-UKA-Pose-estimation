# from ray import tune
import yaml
import numpy as np
from hyperopt import hp, fmin, tpe, STATUS_OK, STATUS_FAIL
import rospy as rp
import time
from subprocess import Popen, PIPE
import yaml
import os
import signal
import pymongo

mongo_client = pymongo.MongoClient("mongodb://mongoadmin:secret@localhost:27888/?authSource=admin") # Careful! This value is also set in the motion analysis config and might lead to inconsistency!
exercises_db = mongo_client.trainerai.exercises

# Put estimates here on how much timebuffer the setup of the test should get and how long we need until the camera node starts and the evaluation begins
TEST_SETUP_TIME_S = 20
CAMERA_NODE_TIMEOUT_S = 10

VALIDATION_REPORT_PATH = '/home/trainerai/trainerai-core/data/validation_report.yml'
BEST_PARAMETERS_PATH = '/home/trainerai/trainerai-core/data/best_parameters.yml'
VALIDATION_TEMP_CONFIG_PATH = '/tmp/ma_validation_config.yml'
STANDARD_CONFIG_PATH = '/home/trainerai/trainerai-core/src/motion_analysis/config.yml'
VIDEO_TIMECODE_PATH = "/home/trainerai/trainerai-core/data/videos/timecodes.yml"

best_scores = dict()

def clean_files():
    try:
        os.remove(VALIDATION_REPORT_PATH)
    except FileNotFoundError:
        pass
    try:
        os.remove(VALIDATION_TEMP_CONFIG_PATH)
    except FileNotFoundError:
        pass

def validation_objective_function(hps):
    status = STATUS_OK
    rp.logerr("Starting Trial with the following parameters:\n\n " + str(hps) + "\n")
    try:
        start_time = time.time()
        clean_files()
        
        old_hps = hps.copy()
        for key, value in old_hps.items():
            if type(value) is dict:
                hps.update(value)
                del hps[key]

        # Read standard config and write new config to temporary file
        with open(STANDARD_CONFIG_PATH, 'r') as infile:
            config = yaml.safe_load(infile)
        config.update(hps)
        config.update({"FORCE_CONFIG": True})

        with open(VALIDATION_TEMP_CONFIG_PATH, 'w') as outfile:
            yaml.dump(config, outfile)

        with open(VIDEO_TIMECODE_PATH) as stream:
            timecode_data = yaml.safe_load(stream)
        # Add some time tome to be sure that the system has enough time for the test setup and outputs a report file
        max_execution_time = timecode_data["sets"][-1]["t_to_s"] + TEST_SETUP_TIME_S + CAMERA_NODE_TIMEOUT_S 

        rp.logerr("Trial will take approximately " + str(max_execution_time/60) + " minutes to finish.")

        process = Popen(['bash', '/home/trainerai/trainerai-core/src/ma_validation/exec_validation.sh', '-t', str(max_execution_time), '-c', VALIDATION_TEMP_CONFIG_PATH], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        poll = process.poll()
        while poll is None:
            time.sleep(1)
            # We poll for the report to be written here
            try:
                with open(VALIDATION_REPORT_PATH, 'r') as infile:
                    report = yaml.safe_load(infile)
                    report = {k: np.array(v) for k, v in report.items()}
                if report != {}: # We are done if the report was written
                    poll = 124
            except Exception:
                pass
            rp.logerr(stdout)
            rp.logerr(stderr)
        else:
            if poll == 124:
                pass
            else:
                rp.logerr(stdout)
                rp.logerr(stderr)
                print("Validation Process returned " + str(poll) + " please investigate!")
                status = STATUS_FAIL

        with open(VALIDATION_REPORT_PATH, 'r') as infile:
            report = yaml.safe_load(infile)
            report = {k: np.array(v) for k, v in report.items()}

        if report == {}:
            rp.logerr("Validation report empty. Something went wrong!")
            status = STATUS_FAIL

        num_exercises = 0
        total_score = 0
        for name, exercise in report.items():
            name = str(name)
            score = exercise[1]
            total_score += score
            num_exercises += 1
            exercise_data = exercises_db.find_one({"name": name})
            if exercise_data and score > exercise_data.get("optimized_config_score", 0):
                exercises_db.update_one({'_id': exercise_data['_id']},{'$set': {'optimized_config_score': score, "optimized_config": config}}, upsert=False)

        score = total_score / min(num_exercises, 1)

        clean_files()
    except Exception as e:
        score = 0
        rp.logerr("Trial error: " + str(e))
        status = STATUS_FAIL

    return {
        'loss': -1.0 * score,
        'status': status,
        # -- store other results like this
        'time_faster_than_max_execution_time': max_execution_time - (time.time() - start_time),
        'evaluation_time': time.time() - start_time,
        }
    

if __name__ == '__main__':
    def handler(signum, frame):
        import rospy as rp
        rp.logerr("Interrupting optimization..")
        os.system("rosnode kill Bagfileplayer")
        os.system("rosnode kill Validator")
        os.system("rosnode kill MotionAnalysis_WorkerHandler")
        exit(0)

    signal.signal(signal.SIGINT, handler)

    space = {
        'num_features_to_progress': hp.choice('num_features_to_progress',
        [
        {
            'ENABLE_NUM_FEATURES_TO_PROGRESS_CHECK': True,
            'NUM_FEATURE_TO_PROGRESS_ALPHA': hp.uniform('NUM_FEATURE_TO_PROGRESS_ALPHA', 3, 6),
            'NUM_FEATURE_TO_PROGRESS_BETA': hp.uniform('NUM_FEATURE_TO_PROGRESS_BETA', 0.1, 1.5),
        },
        {
            'ENABLE_NUM_FEATURES_TO_PROGRESS_CHECK': False
        }
        ]),

        'num_features_progress_too_far': hp.choice('num_features_progress_too_far',
        [
        {
            'ENABLE_NUM_FEATURES_PROGRESSED_TOO_FAR_CHECK': True,
            'NUM_FEATURES_PROGRESSED_TOO_FAR_MU': hp.uniform('NUM_FEATURES_PROGRESSED_TOO_FAR_MU', 0.3, 3),
        },
        {
            'ENABLE_NUM_FEATURES_PROGRESSED_TOO_FAR_CHECK': False
        }
        ]),
        
        'feature_alignment_check': hp.choice('feature_alignment_check',
        [
        {
            'ENABLE_FEATURE_ALIGNMENT_CHECK': True,
            'MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT': hp.uniform('MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT', 0.6, 0.9),
        },
        {
            'ENABLE_FEATURE_ALIGNMENT_CHECK': False
        }
        ]),

        'REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER': hp.uniform('REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER', 0.1, 0.45),
        'REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER': hp.uniform('REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER', 0.1, 0.45),
        'FEATURE_TRAJECTORY_RESOLUTION_FACTOR': hp.uniform('FEATURE_TRAJECTORY_RESOLUTION_FACTOR', 0.03, 0.3),
        'REMOVE_BUMPS_RANGE': hp.quniform('REMOVE_BUMPS_RANGE', 2, 7, 1),
        'JUMPY_PROGRESS_ALPHA': hp.uniform('JUMPY_PROGRESS_ALPHA', 2, 4),
        'JUMPY_PROGRESS_BETA': hp.uniform('JUMPY_PROGRESS_BETA', 3, 5),
        'TRUST_REGION_FILTER_FACTOR': hp.quniform('TRUST_REGION_FILTER_FACTOR', 1, 8, 1),
        'JOINT_DIFFERENCE_FADING_FACTOR': hp.uniform('JOINT_DIFFERENCE_FADING_FACTOR', 0.4, 0.9),
        
    }
    
    best_hps = fmin(validation_objective_function, space, algo=tpe.suggest, max_evals=25)

    rp.logerr("Best Parameters are:")
    rp.logerr(str(best_hps))
    rp.logerr("Saving to " + str(BEST_PARAMETERS_PATH))

    with open(BEST_PARAMETERS_PATH, 'w') as outfile:
            config = yaml.dump(best_hps, outfile)



# First optimization result:
# 'FEATURE_TRAJECTORY_RESOLUTION_FACTOR': 0.020262548446549, 'MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT': 0.801023013123205, 'NUM_FEATURE_TO_PROGRESS_ALPHA': 4.629802767809281, 'NUM_FEATURE_TO_PROGRESS_BETA': 1.4004525185220782, 'REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER': 0.310037868625595, 'REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER': 0.24742207775295433, 'REMOVE_BUMPS_RANGE': 4.0, 'TRUST_REGION_FILTER_FACTOR': 5.0