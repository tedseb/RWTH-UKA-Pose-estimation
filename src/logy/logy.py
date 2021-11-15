import sys, os, errno
import threading
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

if hasattr(sys, '_getframe'):
    currentframe = lambda depth : sys._getframe(depth)
else: #pragma: no cover
    def currentframe(depth : int):
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

def get_message_type(level : int, msg : str, tag : str, module : str, file: str, lineno : int, function : str) :
    message =  {
        "type" : 0,
        "timestamp" : datetime.now().timestamp(),
        "level" : level,
        "msg" : msg,
        "tag" : tag,
        "module" : module,
        "file" : file,
        "lineno" : lineno,
        "function" : function
    }
    return message

class Singleton(type):
    _instance = None
    _lock = threading.Lock()

    def __call__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instance

def debug(msg, tag = "msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, DEBUG, 3)

def info(msg, tag = "msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, INFO, 3)

def warn(msg, tag = "msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, WARNING, 3)

def error(msg, tag = "msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, ERROR, 3)

def critical(msg, tag = "msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, CRITICAL, 3)

def fatal(msg, tag = "msg"):
    logger = Logy()
    logger._root._send_msg(msg, tag, CRITICAL, 3)

def debug_throttle(msg, throttel_time_s, tag = "msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_s, tag, DEBUG, 3)

def info_throttle(msg, throttel_time_s, tag = "msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_s, tag, INFO, 3)

def warn_throttle(msg, throttel_time_s, tag = "msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_s, tag, WARNING, 3)

def error_throttle(msg, throttel_time_s, tag = "msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_s, tag, ERROR, 3)

def critical_throttle(msg, throttel_time_s, tag = "msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_s, tag, CRITICAL, 3)

def fatal_throttle(msg, throttel_time_s, tag = "msg"):
    logger = Logy()
    logger._root._send_msg_throttle(msg, throttel_time_s, tag, CRITICAL, 3)

class LogyHandler:
    def __init__(self, logger, debug_level : int, module : str):
        self._debug_level = debug_level
        self._logger = logger
        self._module = module
        self._throttle_timings = {}

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

    def debug(self, msg, tag = "msg"):
        self._send_msg(msg, tag, DEBUG, 3)

    def info(self, msg, tag = "msg"):
        self._send_msg(msg, tag, INFO, 3)

    def warn(self, msg, tag = "msg"):
        self._send_msg(msg, tag, WARNING, 3)

    def error(self, msg, tag = "msg"):
        self._send_msg(msg, tag, ERROR, 3)

    def critical(self, msg, tag = "msg"):
        self._send_msg(msg, tag, CRITICAL, 3)

    def fatal(self, msg, tag = "msg"):
        self._send_msg(msg, tag, CRITICAL, 3)

    def set_debug_level(self, debug_level : int):
        self._debug_level = debug_level

    def set_module(self, module_name : str):
        self._module = module_name

    def debug_throttle(self, msg, throttel_time_s, tag = "msg"):
        self._send_msg_throttle(msg, throttel_time_s, tag, DEBUG, 3)

    def info_throttle(self, msg, throttel_time_s, tag = "msg"):
        self._send_msg_throttle(msg, throttel_time_s, tag, INFO, 3)

    def warn_throttle(self, msg, throttel_time_s, tag = "msg"):
        self._send_msg_throttle(msg, throttel_time_s, tag, WARNING, 3)

    def error_throttle(self, msg, throttel_time_s, tag = "msg"):
        self._send_msg_throttle(msg, throttel_time_s, tag, ERROR, 3)

    def critical_throttle(self, msg, throttel_time_s, tag = "msg"):
        self._send_msg_throttle(msg, throttel_time_s, tag, CRITICAL, 3)

    def fatal_throttle(self, msg, throttel_time_s, tag = "msg"):
        self._send_msg_throttle(msg, throttel_time_s, tag, CRITICAL, 3)

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

    def _pipe_send(self, data : Dict):
        if self._pipe.closed:
            print("Pipe unexpectedly closed. Try to open file again")
            if not self._open_pipe():
                return
        lock = FileLock(FIFO_LOCK)
        with lock:
            pickle.dump(data, self._pipe, protocol=pickle.HIGHEST_PROTOCOL)
            #self._pipe.write(str(data) + "\n")
            self._pipe.flush()

    def send_msg(self, level : int, msg : str, tag : str, module : str, file: str, lineno : int, function : str):
        with self._lock:
            data = get_message_type(level, msg, tag, module, file, lineno, function)
            self._pipe_send(data)

    def set_root_debug_level(self, debug_level : int):
        self._root.set_debug_level(debug_level)

    def set_root_module(self, module_name : str):
        self._root.set_module(module_name)

    def get_or_create_logger(self, name : str, debug_level = WARNING, module_name = "--") -> LogyHandler:
        if name not in self._logger_dict:
            self._logger_dict[name] = LogyHandler(self, debug_level, module_name)
        return self._logger_dict[name]

    def basic_config(self, debug_level = None, module_name = None):
        if debug_level is not None:
            self.set_root_debug_level(debug_level)
        if module_name is not None:
            self.set_root_module(module_name)

def get_or_create_logger(name : str, debug_level = WARNING, module_name = "--") -> LogyHandler:
    logy_ = Logy()
    return logy_.get_or_create_logger(name, debug_level, module_name)

def set_root_debug_level(debug_level : int):
    logy_ = Logy()
    logy_._root.set_debug_level(debug_level)

def basic_config(debug_level = None, module_name = None):
    logy_ = Logy()
    logy_.basic_config(debug_level, module_name)

def exception_hook(exctype, value, trace):
    traceback_formated = traceback.format_exception(exctype, value, trace)
    traceback_string = "".join(traceback_formated)
    critical("Logy Traceback Hook: \n" + traceback_string)
    sys.__excepthook__(exctype, value, trace)

sys.excepthook = exception_hook
