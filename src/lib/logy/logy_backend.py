#!/usr/bin/python3
import os
import errno
import sys
import time
import signal
import pathlib
import pickle
import threading
import argparse
from typing import Dict, List
from datetime import datetime
from dataclasses import dataclass
import traceback
import neptune.new as neptune
import nvidia_smi
import psutil

NOTSET = 0
TEST = 5
DEBUG = 10
TRACING = 15
INFO = 20
WARNING = 30
ERROR = 40
CRITICAL = 50

MONITORING_UPDATE = 2
NEPTUNE_PROJ = None
LOG_TO_TERMINAL = True
LOG_TO_TERMINAL_LEVEL = DEBUG
LOG_TO_FILE = False
LOG_TO_FILE_LEVEL = DEBUG
DEFAULT_FILE_PREFIX = "/home/trainerai/trainerai-core/data/logs/log"
PIPE_WAIT_TIME = 0.5

#Debug Levels
MESSAGE_OUTPUT_LEVEL_TERMINAL = 20
MESSAGE_OUTPUT_LEVEL_FILE = 90
#0   #
#5   [ERROR]:
#10  [ERROR]::File:
#20  [ERROR]::File:Line:
#30  [ERROR]::Modul:File:Line::
#40  [ERROR]::File::Time:
#50  [ERROR]::File:Line::Time:
#60  [ERROR]::File:Function:
#70  [ERROR]::File:Function:Line:
#80  [ERROR]::File:Function:Line::Time:
#90  [ERROR]::Modul:File:Line::Time:
#100 [ERROR]::Modul:File:Function:Line::Time:
#110 [ERROR]::Modul:File:Function:Line:
#120 [ERROR]::Time::Modul:File:Function:Line:
#130 [ERROR]::Time::File:Line:
#140 [ERROR]::Function:

FIFO = '/home/trainerai/logy_pipe'

class LogType:
    MSG = 0
    AVG = 1
    MEAN = 2
    VARIABLE = 3
    TRACING = 4
    STR = 5

@dataclass
class AverageData:
    caller_hash: hash
    num: int
    last_avg: float

@dataclass
class MeanData:
    caller_hash: hash
    values: List[float]

@dataclass
class VariableData:
    caller_hash: hash
    values: List[float]
    time_stamps: List[float]

_name_to_level = {
    'critical': CRITICAL,
    'error': ERROR,
    'warning': WARNING,
    'info': INFO,
    'tracing': TRACING,
    'debug': DEBUG,
    'notset': NOTSET,
}

class LogyBackend:
    _level_to_name = {
        CRITICAL: 'CRITICAL',
        ERROR: 'ERROR',
        WARNING: 'WARNING',
        INFO: 'INFO',
        TRACING: 'TRACING',
        DEBUG: 'DEBUG',
        NOTSET: 'NOTSET',
    }

    #0=Modul, 1=File, 2=Function, 3=Line, 4=Time, 5=Msg
    _format_strings = {
        0: "{5}",
        5: ": {5}",
        10: "::{1}: {5}",
        20: "::{1}:{3}: {5}",
        30: "::{0}:{1}:{3}: {5}",
        40: "::{1}::{4}: {5}",
        50: "::{1}:{3}::{4}: {5}",
        60: "::{1}:{2}: {5}",
        70: "::{1}:{2}:{3}: {5}",
        80: "::{1}:{2}:{3}::{4}: {5}",
        90: "::{0}:{1}:{3}::{4}: {5}",
        100: "::{0}:{1}:{2}:{3}::{4}: {5}",
        110: "::{0}:{1}:{2}:{3}: {5}",
        120: "::{4}::{0}:{1}:{2}::{3} {5}",
        130: "::{4}::{1}:{3}: {5}",
        140: "::{2}: {5}"
    }

    _format_colors = {
        DEBUG: '\033[94m',
        TRACING: '\033[95m', #'\033[35m'
        INFO: '\033[97m',
        WARNING: '\033[93m',
        ERROR: '\033[91m',
        CRITICAL: '\033[31m',
        999: '\033[0m'
    }

    def __init__(self,
            log_to_terminal=LOG_TO_TERMINAL,
            log_to_terminal_level=LOG_TO_TERMINAL_LEVEL,
            message_output_level_terminal=MESSAGE_OUTPUT_LEVEL_TERMINAL,
            log_to_file=LOG_TO_FILE,
            log_to_file_level =LOG_TO_FILE_LEVEL,
            message_output_level_file=MESSAGE_OUTPUT_LEVEL_FILE,
            file_prefix=DEFAULT_FILE_PREFIX,
            pipe_wait_time=PIPE_WAIT_TIME,
            neptune_proj=NEPTUNE_PROJ,
            test_case = False,
            sub_name = None,
            print_tags=[]):

        self._log_to_terminal = log_to_terminal
        self._log_to_terminal_level = log_to_terminal_level
        self._message_output_level_terminal = message_output_level_terminal
        self._log_to_file = log_to_file
        self._log_to_file_level = log_to_file_level
        self._message_output_level_file = message_output_level_file
        self._pipe_wait_time = pipe_wait_time
        self._use_neptune = neptune_proj != None
        self._neptune_proj = neptune_proj
        self._log_file = None
        self._log_file_path = None
        self._pipe = None
        self._error_occured = 0
        self._neptune_run = None
        self._print_tags = print_tags
        self._avg_data: Dict[AverageData] = {}
        self._mean_data: Dict[MeanData] = {}
        self._var_data: Dict[VariableData] = {}
        self._tracing_data: Dict[VariableData] = {}
        self._test_case = test_case
        self._shutdown = False
        self._sub_name = sub_name

        if self._use_neptune:
            self._log_message(" Logy: Log online with Neptune. neptune.ai", INFO)
            self._log_to_file = True

        if self._test_case:
            self._log_to_file = True
            self._log_to_file_level = TEST
            self._message_output_level_file = 0
            self._open_log_file(file_prefix + "_system_test")
        else:
            if self._log_to_file:
                self._open_log_file(file_prefix)


        log_level = self._level_to_name[log_to_terminal_level]
        self._log_message(f" Logy: Log to terminal with log_level={log_level}", INFO)

    def __del__(self):
        if self._avg_data or self._mean_data:
            self._log_message("### Print all Logy meta data ###", INFO)
            self._log_avg_result()
            self._log_mean_result()

        self._log_message("### Clean Shutdown of Logy Backend ###", INFO)
        if self._pipe is not None:
            self._pipe.close()
        if self._log_file is not None:
            self._log_file.close()

        if self._use_neptune:
            self._neptune_run.stop()

    def _log_avg_result(self):
        avg_logs = {}
        for name, value in self._avg_data.items():
            info_str = f"# Average '{name}'={value.last_avg}, N={value.num}"
            self._log_message(info_str, INFO)
            avg_logs[name] = value.last_avg
            avg_logs[name + "_N"] = value.num

        if self._use_neptune and avg_logs:
            self._neptune_run["average"] = avg_logs

    def _log_mean_result(self):
        mean_logs = {}
        for name, value in self._mean_data.items():
            num = len(value.values)
            mean_id = int(num / 2.0)
            value.values.sort()
            info_str = f"# Mean '{name}'={value.values[mean_id]}, N={num}"
            self._log_message(info_str, INFO)
            mean_logs[name] = value.values[mean_id]
            mean_logs[name + "_N"] = num

        if self._use_neptune and mean_logs:
            self._neptune_run["mean"] = mean_logs

    def _open_pipe(self) -> bool:
        try:
            self._pipe =  open(FIFO, 'rb')
        except OSError:
            trace = traceback.format_exc()
            print("[ERROR] Pipe Error: \n" + trace)
            return False
        return not self._pipe.closed

    def _open_log_file(self, file_prefix):
        date_str = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
        file_name = f"{file_prefix}_{date_str}.txt"
        path = pathlib.Path(file_name).parent
        path.mkdir(parents=True, exist_ok=True)
        self._log_file_path = file_name
        try:
            self._log_file = open(file_name, 'w')
        except OSError as exception:
            self._log_file = None
            trace = traceback.format_exc()
            self._log_message(f" Logy: Can not open Logger File. \n{trace}", CRITICAL)

    def _log_data(self, data: Dict):
        if "type" not in data:
            self._log_message(" Logy: Message does not contain 'type'", ERROR)
            return

        message_type = data["type"]
        if message_type == LogType.MSG:
            self._log_data_message(data)
        elif message_type == LogType.AVG:
            self._log_avg(data)
        elif message_type == LogType.MEAN:
            self._log_mean(data)
        elif message_type == LogType.VARIABLE:
            self._log_var(data)
        elif message_type == LogType.TRACING:
            self._log_tracing(data)
        elif message_type == LogType.STR:
            self._log_str(data)
        else:
            self._log_message(f"Logy: Unknown message type {message_type}", WARNING)

    def _log_avg(self, data: Dict):
        caller_hash = data["hash"]
        name = data["name"]
        value = data["value"]

        values_before: AverageData = self._avg_data.get(name)
        if values_before is None:
            self._avg_data[name] = AverageData(caller_hash, 1, value)
            return

        n = values_before.num
        y = (n / (n + 1)) * values_before.last_avg
        avg = y + (value / (n + 1))
        values_before.last_avg = avg
        values_before.num += 1

        if values_before.caller_hash != caller_hash:
            self._log_message(f" Logy: The avg log with name '{name}' is called from another location", WARNING)

    def _log_mean(self, data: Dict):
        caller_hash = data["hash"]
        name = data["name"]
        value = data["value"]

        values_before: MeanData = self._mean_data.get(name)
        if values_before is None:
            self._mean_data[name] = MeanData(caller_hash, [value])
            return

        values_before.values.append(value)

        if values_before.caller_hash != caller_hash and not self._test_case:
            self._log_message(f" Logy: The Mean log with name '{name}' is called from another location", WARNING)

    def _log_var(self, data: Dict):
        caller_hash = data["hash"]
        name = data["name"]
        value = data["value"]

        if self._use_neptune:
            self._neptune_run[f"variable/{name}"].log(value)

        if "var" in self._print_tags:
            self._log_msg_to_terminal(f"VARIABLE:{name}: {value}", DEBUG)

        values_before: VariableData = self._var_data.get(name)
        if values_before is None:
            self._var_data[name] = VariableData(caller_hash, [value], [time.time()])
            return

        values_before.time_stamps.append(time.time())
        values_before.values.append(value)
        if values_before.caller_hash != caller_hash and not self._test_case:
            self._log_message(f" Logy: The Variable log with name '{name}' is called from another location", WARNING)

    def _log_str(self, data: Dict):
        caller_hash = data["hash"]
        name = data["name"]
        value = data["value"]

        if self._use_neptune:
            self._neptune_run[f"logs/{name}"].log(value)

        if "str" in self._print_tags:
            self._log_msg_to_terminal(f"VARIABLE:{name}: {value}", DEBUG)

        values_before: VariableData = self._var_data.get(name)
        if values_before is None:
            self._var_data[name] = VariableData(caller_hash, [value], [time.time()])
            return

        values_before.time_stamps.append(time.time())
        values_before.values.append(value)
        if values_before.caller_hash != caller_hash and not self._test_case:
            self._log_message(f" Logy: The Variable log with name '{name}' is called from another location", WARNING)

    def _log_tracing(self, data: Dict):
        caller_hash = data["hash"]
        name = data["name"]
        value = data["value"]
        tag = data["tag"]

        if self._use_neptune:
            self._neptune_run[f"tracing/{name}"].log(value)

        if "tracing" in self._print_tags:
            self._log_msg_to_terminal(f"{tag}:{name}: {value}", TRACING)

        values_before: VariableData = self._tracing_data.get(name)
        if values_before is None:
            self._tracing_data[name] = VariableData(caller_hash, [value], [time.time()])
            return

        values_before.time_stamps.append(time.time())
        values_before.values.append(value)
        if values_before.caller_hash != caller_hash and not self._test_case:
            self._log_message(f" Logy: The Variable log with name '{name}' is called from another location", WARNING)

    def _log_data_message(self, data: Dict):
        log_level = data["level"]
        log_message = self._format_message(data, self._message_output_level_terminal)

        if self._use_neptune:
            if log_level == ERROR and self._error_occured <= ERROR:
                self._neptune_run["Info/State"] = "ERROR"
                self._error_occured = ERROR
            if log_level == CRITICAL:
                self._neptune_run["Info/State"] = "CRITICAL"
                self._error_occured = CRITICAL

        if log_level >= self._log_to_terminal_level:
            if data["tag"] == "msg" or data["tag"] in self._print_tags:
                self._log_msg_to_terminal(log_message, log_level)

        if self._message_output_level_terminal != self._message_output_level_file:
            log_message = self._format_message(data, self._message_output_level_file)

        if log_level >= self._log_to_file_level:
            if self._test_case:
                if not (data["tag"] == "test"):
                    return
            self._log_msg_to_file(log_message, log_level)

    def _log_message(self, msg, log_level):
        if log_level >= self._log_to_terminal_level:
            self._log_msg_to_terminal(msg, log_level)
        if log_level >= self._log_to_file_level:
            self._log_msg_to_file(msg, log_level)

    def _format_message(self, data: Dict, format: int):
        msg = data["msg"]
        timestamp = data["timestamp"]
        file = os.path.basename(data["file"])
        lineno = data["lineno"]
        function = data["function"]
        module = data["module"]

        date = datetime.fromtimestamp(timestamp)
        date = date.strftime("%H-%M-%S.%f")[:-3]

        format_str: str = self._format_strings[format]
        msg = format_str.format(module, file, function, lineno, date, msg)
        return msg

    def _log_msg_to_terminal(self, msg: str, log_level: int):
        if self._log_to_terminal:
            color_level_str = f"{self._format_colors[log_level]}[{self._level_to_name[log_level]}]{self._format_colors[999]}"
            if log_level < ERROR:
                print(color_level_str + msg)
            else:
                sys.stderr.write(color_level_str + msg + "\n")

    def _log_msg_to_file(self, msg: str, log_level: int):
        if self._log_to_file and self._log_file is not None:
            level_str = ""
            if log_level >= DEBUG:
                level_str = f"[{self._level_to_name[log_level]}]"
            self._log_file.write(level_str + msg + "\n")

    def pipe_loop(self):
        data = None
        self._open_pipe()
        while True:
            if self._pipe.closed:
                if not self._open_pipe():
                    self._log_message(" Logy: Logger Closed. Can not open Pipe.", CRITICAL)
                    return
            while data is None:
                try:
                    # while True:
                    #     # We don't want to print all the empty lines returned by readline() when no writer have the pipe opened
                    #     data = self._pipe.readline()
                    #     if data:
                    #         print(data)
                    #     else:
                    #         time.sleep(0.1)
                    data = pickle.load(self._pipe)
                except EOFError:
                    data = None
                    time.sleep(self._pipe_wait_time)
                except Exception:
                    trace = traceback.format_exc()
                    self._log_message(f" Pickle Error: \n{trace}", CRITICAL)

            try:
                self._log_data(data)
            except Exception:
                trace = traceback.format_exc()
                self._log_message(f" Logy: Error During Message Parsing. \n{trace}", ERROR)

            data = None

    def _monitoring_thread(self):
        nvidia_smi.nvmlInit()
        gpu_count = nvidia_smi.nvmlDeviceGetCount()
        gpu_handles = []
        for i in range(gpu_count):
            gpu_handles.append(nvidia_smi.nvmlDeviceGetHandleByIndex(i))

        last_time = time.time()
        while not self._shutdown:
            if time.time() - last_time <= MONITORING_UPDATE:
                time.sleep(0.5)
                continue
            last_time = time.time()
            cpu = psutil.cpu_percent()
            ram = psutil.virtual_memory().used / 1000000000
            cpu_temps = psutil.sensors_temperatures()
            cpu_freq = psutil.cpu_freq().current / 1000
            cpu_temp = float(0)
            for temp in cpu_temps["k10temp"]:
                if temp.label == "Tdie":
                    cpu_temp = temp.current

            self._neptune_run["monitoring/digiisland_cpu_util"].log(cpu)
            self._neptune_run["monitoring/digiisland_mem"].log(ram)
            self._neptune_run["monitoring/digiisland_cpu_temperature"].log(cpu_temp)
            self._neptune_run["monitoring/digiisland_cpu_frequence"].log(cpu_freq)

            # print(f"#############################")
            # print(f"# cpu: {cpu}")
            # print(f"# ram: {ram}")
            # print(f"# temp: {cpu_temp}")
            # print(f"# freq: {cpu_freq}")
            for i, handle in enumerate(gpu_handles):
                res_util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle).gpu
                res_mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle).used / 1000000000
                power = nvidia_smi.nvmlDeviceGetPowerUsage(handle) / 1000
                gpu_temp = nvidia_smi.nvmlDeviceGetTemperature(handle, 0)
                self._neptune_run[f"monitoring/digiisland_gpu{i}_util"].log(res_util)
                self._neptune_run[f"monitoring/digiisland_gpu{i}_mem"].log(res_mem)
                self._neptune_run[f"monitoring/digiisland_gpu{i}_power"].log(power)
                self._neptune_run[f"monitoring/digiisland_gpu{i}_temperature"].log(gpu_temp)
            #     print("# GPU mem:", res_mem)
            #     print("# GPU util:", res_util)
            #     print("# GPU temp:", gpu_temp)
            #     print("# GPU Power:", power)
            # print(f"#############################")

    def monitoring_thread(self):
        try:
            self._monitoring_thread()
        except Exception:
            trace = traceback.format_exc()
            self._log_message(f"Error in monitoring_thread. \n{trace}", ERROR)

    def start_reading(self):

        if self._use_neptune:
            self._neptune_run = neptune.init(
                project=f"basti95/{self._neptune_proj}",
                api_token="eyJhcGlfYWRkcmVzcyI6Imh0dHBzOi8vYXBwLm5lcHR1bmUuYWkiLCJhcGlfdXJsIjoiaHR0cHM6Ly9hcHAubmVwdHVuZS5haSIsImFwaV9rZXkiOiJlNGJiNzAyMi0wNmIwLTQxNjctOTRlMi1jNGNlMTEyODc0MDcifQ==",
                source_files=[]
            )
            self._neptune_run["Info"] = {"State": "RUNNING"}
            self._monitoring = threading.Thread(target=self.monitoring_thread, daemon=False)
            self._monitoring.start()

        try:
            self.pipe_loop()
        except KeyboardInterrupt:
            pass

        if self._neptune_run is not None:
            if self._log_to_file:
                self._log_file.flush()
                self._neptune_run["log_file"].upload(self._log_file_path)
                print("upload file")
            if self._error_occured < ERROR:
                self._neptune_run["Info/State"] = "SUCCESS"
            if self._sub_name is not None:
                self._neptune_run["Info/Subname"] = self._sub_name

def main(neptune=None, tags=[], log_level_terminal="warning", test_case=False):
    try:
        os.mkfifo(FIFO)
    except OSError as oe:
        if oe.errno != errno.EEXIST:
            raise

    sub_name = None
    if neptune is not None:
        if neptune.startswith('test'):
            splitted = neptune.split("-")
            neptune = "test"
            if len(splitted) < 2:
                sub_name = "Unknown"
            else:
                sub_name = splitted[1]

    level = _name_to_level[log_level_terminal]
    logger_backend = LogyBackend(neptune_proj=neptune, print_tags=tags, log_to_terminal_level=level, test_case=test_case, sub_name=sub_name)
    # def signal_handler(self, signal):
    #     nonlocal logger_backend
    #     logger_backend._shutdown = True
    #     del logger_backend
    #     sys.exit(0)

    #signal.signal(signal.SIGINT, signal_handler)
    logger_backend.start_reading()
    logger_backend._shutdown = True
    print("EXIT")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--neptune", type=str, help="Start with Neptune logging", default=None)
    parser.add_argument("-t", "--tag", type=str, help="All tags which should be printed on the terminal. e.g: 'msg frame'")
    parser.add_argument("--test", help="Log to test files", action="store_true")
    parser.add_argument("--log-level", type=str, default='warning', help="Debug level", choices=['debug', 'info', 'warning', 'error', 'critical'])

    arg_count = len(sys.argv)
    #print(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    tags = []
    if args.tag is not None and args.tag != "msg":
        tags = str(args.tag).split()


    if args.neptune is not None:
        if args.neptune == 'prod':
            args.neptune = 'test-project'

    test_case = args.test #is not None
    main(args.neptune, tags=tags, log_level_terminal=args.log_level, test_case=test_case)
