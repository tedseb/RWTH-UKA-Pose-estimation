
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
        self._resetted = True
        self._reset_mutex = Lock()

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self.finished.set()

    def run(self):

        while self._resetted:
            with self._reset_mutex:
                self._resetted = False
            self._finished.wait(self.interval)

        if not self.finished.isSet():
            self.function(*self._args, **self._kwargs)
        self._finished.set()

    def reset(self, interval=None):
        """ Reset the timer """

        if interval:
            self.interval = interval

        self.resetted = True
        with self._reset_mutex:
            self.finished.set()
            self.finished.clear()