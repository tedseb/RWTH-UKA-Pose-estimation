
import time
from threading import Thread, Event, Timer, Lock
from multiprocessing import Lock

class ResetTimer(Thread):
    """Call a function after a specified number of seconds:
    t = ResetTimer(30.0, f, args=[], kwargs={})
    t.start() - to start the timer
    t.reset() - to reset the timer
    t.cancel() # stop the timer's action if it's still waiting
    """

    def __init__(self, interval, function, args=[], kwargs={}):
        Thread.__init__(self)
        self._interval = interval
        self._function = function
        self._args = args
        self._kwargs = kwargs
        self._finished = Event()
        self._start = Event()
        self._start.set()
        self._resetted = True
        self._reset_mutex = Lock()
        self._stopped = False
        self._start_time = 0

    def __del__(self):
        self.stop()

    def stop(self):
        self._stopped = True
        self._finished.set()
        self._start.set()

    def join(self):
        self.stop()
        super.join()

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self._finished.set()

    def run(self):
        while not self._stopped:
            self._start.wait()

            while self._resetted:
                with self._reset_mutex:
                    self._resetted = False
                self._start_time = time.time()
                self._finished.wait(self._interval)

            if not self._finished.isSet():
                self._function(*self._args, **self._kwargs)
            self._finished.set()
            self._start.clear()

    def get_elapsed_time(self):
        return time.time() - self._start_time

    def reset(self, interval=None):
        """ Reset the timer """

        if interval is not None:
            self._interval = interval

        self._resetted = True
        with self._reset_mutex:
            self._start.set()
            self._start.clear()
            self._finished.set()
            self._finished.clear()

class ResetTimerHandler():
    def __init__(self, function, args=[], kwargs={}):
        self._function = function
        self._args = args
        self._kwargs = kwargs
        self._timings = {}
        self._timer : ResetTimer = None
        self.running_timing = None
        self._next_time_out_key = None
        self._mutex = Lock()

    def add_timing(self, time, key):
        self._timings[key] = time

        if self._timer is None:
            self._next_time_out_key = key
            self._timer = ResetTimer(time, self.on_time_out)
            self._timer.start()
            return

    def remove_timing(self, key):
        self._timings.pop(key, None)

    def _set_new_timout(self):
        with self._mutex:
            if not self._timings:
                return

            if self._timer is not None:
                elapsed = self._timer.get_elapsed_time()
                for key in self._timings:
                    self._timings[key] -= elapsed
                    assert self._timings[key] > 0

                self._next_time_out_key = min(self._timings, key=self._timings.get)
                self._timer.reset(self._timings[self._next_time_out_key])
            else:
                assert self._timings
                self._next_time_out_key = next(iter(self._timings))
                self._timer = ResetTimer(self._timings[self._next_time_out_key], self._on_time_out)
                self._timer.start()
                return

    def _on_time_out(self):
        timing = self._timings.pop(self._next_time_out_key, None)
        if timing is not None:
            self._function(self._next_time_out_key, *self._args, **self._kwargs)

        if self._timings:
            self._set_new_timout()
        else:
            del self._timer
            self._timer = None