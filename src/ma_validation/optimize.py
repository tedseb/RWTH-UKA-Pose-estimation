# from ray import tune
import yaml
import numpy as np
from hyperopt import hp, fmin, tpe, STATUS_OK, STATUS_FAIL
import rospy as rp
import time
from subprocess import Popen, PIPE
import yaml
import os

# Put estimates here on how much timebuffer the setup of the test should get and how long we need until the camera node starts and the evaluation begins
TEST_SETUP_TIME_S = 20
CAMERA_NODE_TIMEOUT_S = 10

VALIDATION_REPORT_PATH = '/home/trainerai/trainerai-core/data/validation_report.yml'
VALIDATION_TEMP_CONFIG_PATH = '/tmp/ma_validation_config.yml'
STANDARD_CONFIG_PATH = '/home/trainerai/trainerai-core/src/motion_analysis/config.yml'
VIDEO_TIMECODE_PATH = "/home/trainerai/trainerai-core/data/videos/timecodes.yml"


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
    rp.logerr("Starting Trial with the following parameters:\n " + str(hps) + "\n")
    try:
        start_time = time.time()
        clean_files()
        
        # Read standard config and write new config to temporary file
        with open(STANDARD_CONFIG_PATH, 'r') as infile:
            config = yaml.safe_load(infile)
        config.update(hps)
        with open(VALIDATION_TEMP_CONFIG_PATH, 'w') as outfile:
            config = yaml.dump(config, outfile)

        with open(VIDEO_TIMECODE_PATH) as stream:
            timecode_data = yaml.safe_load(stream)
        # Add some time tome to be sure that the system has enough time for the test setup and outputs a report file
        max_execution_time = timecode_data["sets"][-1]["t_to_s"] + TEST_SETUP_TIME_S + CAMERA_NODE_TIMEOUT_S 

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

        tpr_by_fpr = 0
        num_exercises = 0
        for exercise in report.values():
            tpr_by_fpr +=  exercise[5] / exercise[6] if exercise[6] else 0
            num_exercises += 1
        score = tpr_by_fpr / num_exercises

        tpr = exercise[5]
        fpr = exercise[6]

        clean_files()
    except Exception as e:
        score = 0
        tpr = 0
        fpr = 1
        rp.logerr("Trial error: " + str(e))
        status = STATUS_FAIL

    return {
        'loss': -score,
        'status': status,
        # -- store other results like this
        'time_faster_than_max_execution_time': max_execution_time - (time.time() - start_time),
        'evaluation_time': time.time() - start_time,
        'tpr': tpr,
        'fpr': fpr
        }
    

if __name__ == '__main__':
    space = {
        'REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER': hp.uniform('REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER', 0.2, 0.5),
        'REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER': hp.uniform('REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER', 0.2, 0.5)
    }
    
    best_hps = fmin(validation_objective_function, space, algo=tpe.suggest, max_evals=4)

    rp.logerr(str(best_hps))