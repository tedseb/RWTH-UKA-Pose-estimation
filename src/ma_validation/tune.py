# from ray import tune
import yaml

def validation_trainable(config, reporter):
    import time
    from subprocess import Popen, PIPE
    import yaml
    import os

    try:
        os.remove('/home/trainerai/trainerai-core/data/validation_report.yml')
    except FileNotFoundError:
        pass

    process = Popen(['bash', '/home/trainerai/trainerai-core/src/ma_validation/exec_validation.sh', '-t', str(config['max_execution_time'])], stdout=PIPE, stderr=PIPE)
    stdout, stderr = process.communicate()
    poll = process.poll()
    while poll is None:
        time.sleep(0.1)
        print(stdout)
        print(stderr)
    else:
        if poll == 124:
            pass
        else:
            print("Validation Process returned " + str(poll) + " please investigate!")
            return

    with open('/home/trainerai/trainerai-core/data/validation_report.yml', 'r') as infile:
        report = yaml.load(infile)

    if report == {}:
        print("Validation report empty. Something went wrong!")

    total_error = 0
    for k, exercise in report:
        print(exercise)
    

if __name__ == '__main__':
    
    with open("/home/trainerai/trainerai-core/data/videos/timecodes.yml") as stream:
        try:
            timecode_data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Get the time of the last set to estimate when to stop an evaluation run, add 5 seconds for system setup
    test_testup_time = 20
    camera_node_timeout = 10
    max_execution_time = 60 # timecode_data["sets"][-1]["t_to_s"] + test_testup_time + camera_node_timeout # Add some time tome to be sure that the system has enough time for the test setup and outputs a report file
    config = {"max_execution_time": max_execution_time}
    validation_trainable(config, None)


# tune.register_trainable("my_func", my_func)
# ray.init()

# tune.run_experiments({
#     "my_experiment": {
#         "run": "my_func",
#         "stop": { "mean_accuracy": 100 },
#         "config": {
#             "alpha": tune.grid_search([0.2, 0.4, 0.6]),
#             "beta": tune.grid_search([1, 2]),
#         }
#     }
# })

