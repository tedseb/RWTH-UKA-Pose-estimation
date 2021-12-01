import sys, os, errno
import threading
from enum import Enum
from datetime import datetime
from typing import Dict
import pickle
import time
import traceback
from filelock import FileLock

FIFO = '/home/trainerai/logy_pipe'
FIFO_LOCK = '/home/trainerai/logy_pipe.lock'

NOTSET = 0
DEBUG = 10
INFO = 20
WARNING = 30
ERROR = 40
CRITICAL = 50

class LogType:
    MSG = 0
    AVG = 1
    MEAN = 2
    VARIABLE = 3
    TRACING = 4

if hasattr(sys, '_getframe'):
    currentframe = lambda depth: sys._getframe(depth)
else: #pragma: no cover
    def currentframe(depth: int):
        """Return the frame object for the caller's stack frame."""
        try:
            raise Exception
        except Exception:
            return sys.exc_info()[depth].tb_frame.f_back

def findCaller(depth = 1):
    """
    Find the stack frame of the caller so that we can note the source
    file name, line number and function name.
    """
    frame = currentframe(depth)
    if frame is not None:
        frame = frame.f_back

    f_code = frame.f_code
    return (f_code.co_filename, frame.f_lineno, f_code.co_name)

def get_message_type(level: int, msg: str, tag: str, module: str, file: str, lineno: int, function: str) :
    message =  {
        "type": LogType.MSG,
        "timestamp": datetime.now().timestamp(),
        "level": level,
        "msg": msg,
        "tag": tag,
        "module": module,
        "file": file,
        "lineno": lineno,
        "function": function
    }
    return message

def get_value_type(name: str, value: float, tag: str, caller_hash: hash, type: LogType) :
    message =  {
        "type": type,
        "name": name,
        "value": value,
        "tag": tag,
        "hash": caller_hash
    }
    return message

def get_avg_type(name: str, value: float, tag: str, caller_hash: hash) :
    return get_value_type(name, value, caller_hash, tag, LogType.AVG)

def get_mean_type(name: str, value: float, tag: str, caller_hash: hash) :
    return get_value_type(name, value, caller_hash, tag, LogType.MEAN)

def get_var_type(name: str, value: float, tag: str, caller_hash: hash) :
    return get_value_type(name, value, caller_hash, tag, LogType.VARIABLE)

def get_tracing_type(name: str, value: float, tag: str, caller_hash: hash) :
    return get_value_type(name, value, tag, caller_hash, LogType.TRACING)

class Singleton(type):
    _instance = None
    _lock = threading.Lock()

    def __call__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instance

class LogyHandler:
    def __init__(self, logger, debug_level: int, module: str):
        self._debug_level = debug_level
        self._logger = logger
        self._module = module
        self._throttle_timings = {}
        self._variables = {}
        self._tracings = {}
        self._fps_timings = {}

    def _send_msg(self, msg, tag, debug_level, trace_level):
        if debug_level < self._debug_level:
            return
        file_name, line_no, function_name = findCaller(trace_level)
        self._logger.send_msg(debug_level, msg, tag, self._module, file_name, line_no, function_name)

    def _send_msg_throttle(self, msg, throttel_time, tag, debug_level, trace_level):
        if debug_level < self._debug_level:
            return
        file_name, line_no, function_name = findCaller(trace_level)
        throttle_hash = hash(file_name + str(line_no) + function_name)
        throttle_timing = self._throttle_timings.get(throttle_hash)
        timstamp_now = time.time() * 1000

        if throttle_timing is None or timstamp_now - throttle_timing >= throttel_time:
            self._logger.send_msg(debug_level, msg, tag, self._module, file_name, line_no, function_name)
            self._throttle_timings[throttle_hash] = timstamp_now

    def _log_var(self, name: str, value: float, period, smoothing, trace_level=4):
        if period == 0 and smoothing == 0.0:
            self._logger.send_var(name, value, trace_level)

        var_list = self._variables.get(name)
        if var_list is None:
            self._variables[name] = [0, value]
            return

        if smoothing != 0.0:
            val = self._smooth_var(var_list, value, period, smoothing)
        else:
            val = self._compute_new_avg(var_list, value, period)

        if val is not None:
            self._logger.send_var(name, val, trace_level)

    def _log_tracing(self, name: str, value: float, period, smoothing, trace_level=4):
        if period == 0:
            self._logger.send_tracing(name, value, "time", trace_level)

        var_list = self._tracings.get(name)
        if var_list is None:
            self._tracings[name] = [0, value]
            return

        if smoothing != 0.0:
            val = self._smooth_var(var_list, value, period, smoothing)
        else:
            val = self._compute_new_avg(var_list, value, period)

        if val is not None:
            self._logger.send_tracing(name, val, "time", trace_level)

    def _smooth_var(self, var_list, value, period, smoothing):
        per = var_list[0] + 1
        last_val = var_list[1]
        last_val = last_val * smoothing + value * (1.0 - smoothing)
        var_list[1] = last_val
        if per >= period:
             var_list[0] = 0
             return last_val
        else:
            var_list[0] = per
            return None

    def _compute_new_avg(self, old_avg_list, value, period):
        per = old_avg_list[0]
        avg_before = old_avg_list[1]
        y = (per / (per + 1)) * avg_before
        avg = y + (value / (per + 1))
        per += 1
        if per >= period:
             old_avg_list[0] = 0
             return avg
        else:
            old_avg_list[1] = avg
            old_avg_list[0] += 1
            return None

    def _log_fps(self, name, period):
        fps_last_time = self._fps_timings.get(name)
        if fps_last_time is None:
            self._fps_timings[name] = [time.time(), 1]
            return

        fps_last_time[1] += 1
        if fps_last_time[1] < period:
            return

        now = time.time()
        time_elapsed = now - fps_last_time[0]
        fps = fps_last_time[1] / time_elapsed
        fps_last_time[0] = now
        fps_last_time[1] = 1
        self._logger.send_tracing(name, fps, "fps", 4)

    def debug(self, msg: str, tag="msg"):
        self._send_msg(msg, tag, DEBUG, 3)

    def info(self, msg: str, tag="msg"):
        self._send_msg(msg, tag, INFO, 3)

    def warn(self, msg: str, tag="msg"):
        self._send_msg(msg, tag, WARNING, 3)

    def error(self, msg: str, tag="msg"):
        self._send_msg(msg, tag, ERROR, 3)

    def critical(self, msg: str, tag="msg"):
        self._send_msg(msg, tag, CRITICAL, 3)

    def fatal(self, msg: str, tag="msg"):
        self._send_msg(msg, tag, CRITICAL, 3)

    def set_debug_level(self, debug_level: int):
        self._debug_level = debug_level

    def set_module(self, module_name: str):
        self._module = module_name

    def debug_throttle(self, msg: str, throttel_time_ms, tag="msg"):
        self._send_msg_throttle(msg, throttel_time_ms, tag, DEBUG, 3)

    def info_throttle(self, msg: str, throttel_time_ms, tag="msg"):
        self._send_msg_throttle(msg, throttel_time_ms, tag, INFO, 3)

    def warn_throttle(self, msg: str, throttel_time_ms, tag="msg"):
        self._send_msg_throttle(msg, throttel_time_ms, tag, WARNING, 3)

    def error_throttle(self, msg: str, throttel_time_ms, tag="msg"):
        self._send_msg_throttle(msg, throttel_time_ms, tag, ERROR, 3)

    def critical_throttle(self, msg: str, throttel_time_ms, tag="msg"):
        self._send_msg_throttle(msg, throttel_time_ms, tag, CRITICAL, 3)

    def fatal_throttle(self, msg: str, throttel_time_ms, tag="msg"):
        self._send_msg_throttle(msg, throttel_time_ms, tag, CRITICAL, 3)

    def log_avg(self, name: str, value: float):
        self._logger.send_avg(name, value)

    def log_mean(self, name: str, value: float):
        self._logger.send_mean(name, value)

    def log_var(self, name: str, value: float, period=0, smoothing=0.0):
        self._log_var(name, value, period, smoothing)

    def log_fps(self, name: str, period=50):
        self._log_fps(name, period)

class Logy(metaclass=Singleton):
    def __init__(self):
        print("Constructor: Init Logy Writer")
        self._lock = threading.Lock()
        self._root = LogyHandler(self, WARNING, "--")
        self._logger_dict = {}
        self._open_pipe()

    def __del__(self):
        print("Destructor: Close Logy Writer")
        trace = traceback.format_exc()
        print(trace)
        self._pipe.close()

    def _open_pipe(self) -> bool:
        print("OPEN PIPE")
        try:
            os.mkfifo(FIFO)
        except OSError as oe:
            if oe.errno != errno.EEXIST:
                raise

        try:
            self._pipe =  open(FIFO, 'wb')
        except OSError:
            print("Pipe Error")
            return False
        return not self._pipe.closed

    def _pipe_send(self, data: Dict):
        if self._pipe.closed:
            print("Pipe unexpectedly closed. Try to open file again")
            if not self._open_pipe():
                return
        lock = FileLock(FIFO_LOCK)
        with lock:
            pickle.dump(data, self._pipe, protocol=pickle.HIGHEST_PROTOCOL)
            #self._pipe.write(str(data) + "\n")
            self._pipe.flush()

    def send_msg(self, level: int, msg: str, tag: str, module: str, file: str, lineno: int, function: str):
        with self._lock:
            data = get_message_type(level, msg, tag, module, file, lineno, function)
            self._pipe_send(data)

    def send_avg(self, name: str, value: float, trace_level = 3):
        file_name, line_no, function_name = findCaller(trace_level)
        caller_hash = hash(file_name + str(line_no) + function_name)
        with self._lock:
            data = get_avg_type(name, value, "avg", caller_hash)
            self._pipe_send(data)

    def send_mean(self, name: str, value: float, trace_level = 3):
        file_name, line_no, function_name = findCaller(trace_level)
        caller_hash = hash(file_name + str(line_no) + function_name)
        with self._lock:
            data = get_mean_type(name, value, "mean", caller_hash)
            self._pipe_send(data)

    def send_var(self, name: str, value: float, trace_level = 3):
        file_name, line_no, function_name = findCaller(trace_level)
        caller_hash = hash(file_name + str(line_no) + function_name)
        with self._lock:
            data = get_var_type(name, value, "var", caller_hash)
            self._pipe_send(data)

    def send_tracing(self, name: str, value: float, tag: str, trace_level = 3):
        file_name, line_no, function_name = findCaller(trace_level)
        caller_hash = hash(file_name + str(line_no) + function_name)

        with self._lock:
            data = get_tracing_type(name, value, tag, caller_hash)
            self._pipe_send(data)

    def set_root_debug_level(self, debug_level: int):
        self._root.set_debug_level(debug_level)

    def set_root_module(self, module_name: str):
        self._root.set_module(module_name)

    def get_or_create_logger(self, name: str, debug_level=WARNING, module_name="--") -> LogyHandler:
        if name not in self._logger_dict:
            self._logger_dict[name] = LogyHandler(self, debug_level, module_name)
        return self._logger_dict[name]

    def basic_config(self, debug_level=None, module_name=None):
        if debug_level is not None:
            self.set_root_debug_level(debug_level)
        if module_name is not None:
            self.set_root_module(module_name)

def get_or_create_logger(name: str, debug_level=WARNING, module_name="--") -> LogyHandler:
    logy_ = Logy()
    return logy_.get_or_create_logger(name, debug_level, module_name)

def set_root_debug_level(debug_level: int):
    logy_ = Logy()
    logy_._root.set_debug_level(debug_level)

def basic_config(debug_level=None, module_name=None):
    logy_ = Logy()
    logy_.basic_config(debug_level, module_name)

def exception_hook(exctype, value, trace):
    traceback_formated = traceback.format_exception(exctype, value, trace)
    traceback_string = "".join(traceback_formated)
    critical("Logy Traceback Hook: \n" + traceback_string)
    sys.__excepthook__(exctype, value, trace)

def debug(msg: str, tag="msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, DEBUG, 3)

def info(msg: str, tag="msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, INFO, 3)

def warn(msg: str, tag="msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, WARNING, 3)

def error(msg: str, tag="msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, ERROR, 3)

def critical(msg: str, tag="msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, CRITICAL, 3)

def fatal(msg: str, tag="msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, CRITICAL, 3)

def debug_throttle(msg: str, throttel_time_ms, tag="msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_ms, tag, DEBUG, 3)

def info_throttle(msg: str, throttel_time_ms, tag="msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_ms, tag, INFO, 3)

def warn_throttle(msg: str, throttel_time_ms, tag="msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_ms, tag, WARNING, 3)

def error_throttle(msg: str, throttel_time_ms, tag="msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_ms, tag, ERROR, 3)

def critical_throttle(msg: str, throttel_time_ms, tag="msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_ms, tag, CRITICAL, 3)

def fatal_throttle(msg: str, throttel_time_ms, tag="msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_ms, tag, CRITICAL, 3)

def log_avg(name: str, value: float):
    logger = Logy()
    logger.send_avg(name, value)

def log_mean(name: str, value: float):
    logger = Logy()
    logger.send_mean(name, value)

def log_var(name: str, value: float, period=0, smoothing=0.0):
    '''smoothing: value = (last_value * smoothing) + (value * (1.0-smoothing))'''
    logger = Logy()
    logger._root._log_var(name, value, period, smoothing)

def log_fps(name: str, period=50):
    logger = Logy()
    logger._root._log_fps(name, period)

def trace_time(name, period=50, smoothing=0.9):
    def trace_time_decorator(func):
        def _trace_time(*args, **kwargs):
            time_elapsed = time.time()
            val = func(*args, **kwargs)
            time_elapsed = (time.time() - time_elapsed) * 1000
            logger = Logy()
            logger._root._log_tracing(name, time_elapsed, period, smoothing, 6)
            return val
        return _trace_time
    return trace_time_decorator

class TraceTime:
    def __init__(self, name, period=50, smoothing=0.9):
        self._name = name
        self._period = 50
        self._smoothing = smoothing
        self._time_stamp = 0

    def __enter__(self):
        self._time_stamp = time.time()

    def __exit__(self, type, value, traceback):
        time_elapsed = (time.time() - self._time_stamp) * 1000
        logger = Logy()
        logger._root._log_tracing(self._name, time_elapsed, self._period, self._smoothing, 5)

def catch_ros(func):
    def _catch_ros(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            traceback_string = traceback.format_exc()
            critical_throttle("Logy Traceback Hook (ROS CALLBACK): \n" + traceback_string, 1000)
    return _catch_ros

def catch_thread(func):
    def _catch_thread(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            traceback_string = traceback.format_exc()
            critical_throttle("Logy Traceback Hook (THREAD): \n" + traceback_string, 1000)
    return _catch_thread

def catch_thread_and_restart(func):
    def _catch_catch_thread_and_restart(*args, **kwargs):
        while True:
            try:
                return func(*args, **kwargs)
            except Exception:
                traceback_string = traceback.format_exc()
                error_throttle("Logy Traceback Hook (THREAD): \n" + traceback_string + "\nRestart Thread", 1000)
    return _catch_catch_thread_and_restart

sys.excepthook = exception_hook
