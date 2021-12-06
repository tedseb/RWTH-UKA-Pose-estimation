# from ray import tune
import yaml
import numpy as np
from hyperopt import hp
from hyperopt import fmin, tpe
import rospy as rp

def validation_objective_function(hps):
    import time
    from subprocess import Popen, PIPE
    import yaml
    import os

    # Get the time of the last set to estimate when to stop an evaluation run, add 5 seconds for system setup
    test_testup_time = 20
    camera_node_timeout = 10
    max_execution_time = timecode_data["sets"][-1]["t_to_s"] + test_testup_time + camera_node_timeout # Add some time tome to be sure that the system has enough time for the test setup and outputs a report file
    config = {"max_execution_time": max_execution_time}

    try:
        os.remove('/home/trainerai/trainerai-core/data/validation_report.yml')
    except FileNotFoundError:
        pass

    try:
        os.remove('/tmp/ma_validation_config.yml')
    except FileNotFoundError:
        pass
    
    with open('/tmp/ma_validation_config.yml', 'w') as outfile:
        yaml.dump(config, outfile)

    process = Popen(['bash', '/home/trainerai/trainerai-core/src/ma_validation/exec_validation.sh', '-t', str(config['max_execution_time']), '-c', "/tmp/ma_validation_config.yml"], stdout=PIPE, stderr=PIPE)
    stdout, stderr = process.communicate()
    poll = process.poll()
    while poll is None:
        time.sleep(1)
        rp.logerr(stdout)
        rp.logerr(stderr)
    else:
        if poll == 124:
            pass
        else:
            print("Validation Process returned " + str(poll) + " please investigate!")
            return

    try:
        os.remove('/tmp/ma_validation_config.yml')
    except FileNotFoundError:
        pass

    with open('/home/trainerai/trainerai-core/data/validation_report.yml', 'r') as infile:
        report = yaml.safe_load(infile)
        report = {k: np.array(v) for k, v in report.items()}

    if report == {}:
        rp.logerr("Validation report empty. Something went wrong!")

    tpr_by_fpr = 0
    num_exercises = 0
    for exercise in report.values():
        tpr_by_fpr +=  exercise[5] / exercise[6] if exercise[6] else 1
        num_exercises += 1
    score = tpr_by_fpr / num_exercises

    return -score
    

if __name__ == '__main__':
    with open("/home/trainerai/trainerai-core/data/videos/timecodes.yml") as stream:
        try:
            timecode_data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    space = hp.choice('a',
        [
            ('case 1', 1 + hp.lognormal('c1', 0, 1)),
            ('case 2', hp.uniform('c2', -10, 10))
        ])
    
    best_hps = fmin(validation_objective_function, space, algo=tpe.suggest, max_evals=100)

    rp.logerr(str(best_hps))