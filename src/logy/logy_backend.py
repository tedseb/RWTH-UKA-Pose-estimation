#!/usr/bin/python3
import os
import errno
import sys
import time
import pathlib
import pickle
from typing import Dict
from datetime import datetime
import traceback
import neptune.new as neptune

NOTSET = 0
DEBUG = 10
INFO = 20
WARNING = 30
ERROR = 40
CRITICAL = 50

USE_NEPTUNE = False
LOG_TO_TERMINAL = True
LOG_TO_TERMINAL_LEVEL = DEBUG
LOG_TO_FILE = True
LOG_TO_FILE_LEVEL = DEBUG
DEFAULT_FILE_PREFIX = "/home/trainerai/trainerai-core/data/logs/log"
PIPE_WAIT_TIME = 0.5

#Debug Levels
MESSAGE_OUTPUT_LEVEL_TERMINAL = 30
MESSAGE_OUTPUT_LEVEL_FILE = 90
#0   [ERROR]:
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

FIFO = '/home/trainerai/logy_pipe'

class LogyBackend:
    _level_to_name = {
        CRITICAL: 'CRITICAL',
        ERROR: 'ERROR',
        WARNING: 'WARNING',
        INFO: 'INFO',
        DEBUG: 'DEBUG',
        NOTSET: 'NOTSET',
    }

    #0=Modul, 1=File, 2=Function, 3=Line, 4=Time, 5=Msg
    _format_strings = {
        0: ": {5}",
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
        130: "::{4}::{1}:{3}: {5}"
    }

    _format_colors = {
        DEBUG: '\033[94m',
        INFO: '\033[97m',
        WARNING: '\033[93m',
        ERROR: '\033[91m',
        CRITICAL: '\033[31m',
        999: '\033[0m'
    }

    def __init__(self,
            log_to_terminal = LOG_TO_TERMINAL,
            log_to_terminal_level = LOG_TO_TERMINAL_LEVEL,
            message_output_level_terminal = MESSAGE_OUTPUT_LEVEL_TERMINAL,
            log_to_file = LOG_TO_FILE,
            log_to_file_level =LOG_TO_FILE_LEVEL,
            message_output_level_file = MESSAGE_OUTPUT_LEVEL_FILE,
            file_prefix = DEFAULT_FILE_PREFIX,
            pipe_wait_time = PIPE_WAIT_TIME,
            use_neptune = USE_NEPTUNE):

        self._log_to_terminal = log_to_terminal
        self._log_to_terminal_level = log_to_terminal_level
        self._message_output_level_terminal = message_output_level_terminal
        self._log_to_file = log_to_file
        self._log_to_file_level = log_to_file_level
        self._message_output_level_file = message_output_level_file
        self._pipe_wait_time = pipe_wait_time
        self._use_neptune = use_neptune
        self._log_file = None
        self._log_file_path = None
        self._pipe = None
        self._error_occured = 0
        self._neptune_run = None
        if self._log_to_file:
            self._open_log_file(file_prefix)

    def __del__(self):
        self._log_message("Logy: Clean Shutdown of Logy Backend", INFO)
        if self._pipe is not None:
            self._pipe.close()
        if self._log_file is not None:
            self._log_file.close()

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
            self._log_message(f"Can not open Logger File. \n{trace}", CRITICAL)

    def _log_data(self, data : Dict):
        if "type" not in data:
            self._log_message("Message does not contain 'type'", ERROR)
            return

        if data["type"] == 0:
            self._log_data_message(data)

    def _log_data_message(self, data : Dict):
        debug_level = data["level"]
        log_message = self._format_message(data, self._message_output_level_terminal)

        if self._use_neptune:
            if debug_level == ERROR and self._error_occured <= ERROR:
                self._neptune_run["Info"] = {"State" : "ERROR"}
                self._error_occured = ERROR
            if debug_level == CRITICAL:
                self._neptune_run["Info"] = {"State" : "CRITICAL"}
                self._error_occured = CRITICAL

        if debug_level >= self._log_to_terminal_level:
            self._log_msg_to_terminal(log_message, debug_level)

        if self._message_output_level_terminal != self._message_output_level_file:
            log_message = self._format_message(data, self._message_output_level_file)

        if debug_level >= self._log_to_file_level:
            self._log_msg_to_file(log_message, debug_level)

    def _log_message(self, msg, debug_level):
        if debug_level >= self._log_to_terminal_level:
            self._log_msg_to_terminal(msg, debug_level)
        if debug_level >= self._log_to_file_level:
            self._log_msg_to_file(msg, debug_level)

    def _format_message(self, data : Dict, format : int):
        msg = data["msg"]
        timestamp = data["timestamp"]
        file = os.path.basename(data["file"])
        lineno = data["lineno"]
        function = data["function"]
        module = data["module"]

        date = datetime.fromtimestamp(timestamp)
        date = date.strftime("%H-%M-%S")

        format_str : str = self._format_strings[format]
        msg = format_str.format(module, file, function, lineno, date, msg)
        return msg

    def _log_msg_to_terminal(self, msg : str, debug_level : int):
        if self._log_to_terminal:
            color_level_str = f"{self._format_colors[debug_level]}[{self._level_to_name[debug_level]}]{self._format_colors[999]}"
            if debug_level < ERROR:
                print(color_level_str + msg)
            else:
                sys.stderr.write(color_level_str + msg + "\n")

    def _log_msg_to_file(self, msg : str, debug_level : int):
        if self._log_to_file and self._log_file is not None:
            level_str = f"[{self._level_to_name[debug_level]}]"
            self._log_file.write(level_str + msg + "\n")

    def pipe_loop(self):
        data = None
        self._open_pipe()
        print("Logy Backend is listening")
        while True:
            if self._pipe.closed:
                if not self._open_pipe():
                    self._log_message("Logy: Logger Closed. Can not open Pipe.", CRITICAL)
                    return
            while data is None:
                try:
                    data = pickle.load(self._pipe)
                except EOFError:
                    data = None
                    time.sleep(self._pipe_wait_time)

            try:
                self._log_data(data)
            except Exception:
                trace = traceback.format_exc()
                self._log_message(f"Logy: Error During Message Parsing. \n{trace}", ERROR)

            data = None

    def start_reading(self):

        if self._use_neptune:
            self._neptune_run = neptune.init(
                project="basti95/test-project",
                api_token="eyJhcGlfYWRkcmVzcyI6Imh0dHBzOi8vYXBwLm5lcHR1bmUuYWkiLCJhcGlfdXJsIjoiaHR0cHM6Ly9hcHAubmVwdHVuZS5haSIsImFwaV9rZXkiOiJlNGJiNzAyMi0wNmIwLTQxNjctOTRlMi1jNGNlMTEyODc0MDcifQ==",
                source_files=[]
            )
            self._neptune_run["Info"] = {"State" : "RUNNING"}

        try:
            self.pipe_loop()
        except KeyboardInterrupt:
            pass

        if self._neptune_run is not None:
            if self._log_to_file:
                self._neptune_run["log_file"].upload(self._log_file_path)
            if self._error_occured < ERROR:
                self._neptune_run["Info"] = {"State" : "SUCCESS"}
            self._neptune_run.stop()

def main():
    try:
        os.mkfifo(FIFO)
    except OSError as oe:
        if oe.errno != errno.EEXIST:
            raise

    logger_backend = LogyBackend()
    logger_backend.start_reading()

if __name__ == '__main__':
    main()
