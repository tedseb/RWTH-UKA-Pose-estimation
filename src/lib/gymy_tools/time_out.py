import ctypes
import threading

def async_raise(target_tid, exception):
    """Raises an asynchronous exception in another thread.
    Read http://docs.python.org/c-api/init.html#PyThreadState_SetAsyncExc
    for further enlightenments.
    :param target_tid: target thread identifier
    :param exception: Exception class to be raised in that thread
    """
    # Ensuring and releasing GIL are useless since we're not in C
    # gil_state = ctypes.pythonapi.PyGILState_Ensure()
    ret = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(target_tid),
                                                     ctypes.py_object(exception))
    # ctypes.pythonapi.PyGILState_Release(gil_state)
    if ret == 0:
        raise ValueError("Invalid thread ID {}".format(target_tid))
    elif ret > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(target_tid), None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

class BaseTimeout(object):
    def __init__(self, seconds):
        self._seconds = seconds
        self._interupted = False

    def __bool__(self):
        return not self._interupted

    def not_interupted(self):
        return not self._interupted

    def __enter__(self):
        self.setup_interrupt()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.suppress_interrupt()
        return True

    def cancel(self):
        self.suppress_interrupt()

    def suppress_interrupt(self):
        raise NotImplementedError

    def setup_interrupt(self):
        raise NotImplementedError

class ThreadingTimeout(BaseTimeout):
    def __init__(self, seconds):
        super(ThreadingTimeout, self).__init__(seconds)
        self.target_tid = threading.current_thread().ident
        self.timer = None  # PEP8

    def stop(self):
        self._interupted = True
        async_raise(self.target_tid, TimeoutError)

    def setup_interrupt(self):
        self.timer = threading.Timer(self._seconds, self.stop)
        self.timer.start()

    def suppress_interrupt(self):
        self.timer.cancel()

import signal
class SignalTimeout(BaseTimeout):
    '''Faster but does only work in main thread'''
    def __init__(self, seconds):
        super(SignalTimeout, self).__init__(seconds)

    def handle_timeout(self, signum, frame):
        self._interupted = True
        raise TimeoutError

    def setup_interrupt(self):
        signal.signal(signal.SIGALRM, self.handle_timeout)
        signal.setitimer(signal.ITIMER_REAL, self._seconds)

    def suppress_interrupt(self):
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, signal.SIG_DFL)


def get_timout_class() -> BaseTimeout:
    if threading.current_thread().__class__.__name__ == '_MainThread':
        return SignalTimeout
    return ThreadingTimeout