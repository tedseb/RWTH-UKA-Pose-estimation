
from threading import Thread, Event, Timer
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
                self._finished.wait(self._interval)

            if not self._finished.isSet():
                self._function(*self._args, **self._kwargs)
            self._finished.set()
            self._start.clear()

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