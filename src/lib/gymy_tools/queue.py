from multiprocessing import Lock
from collections import deque

class Queue:
    def __init__(self, queue_len = 10):
        self._elements = deque([])
        self._mutex = Lock()
        self._queue_len = queue_len

    def put(self, item):
        with self._mutex:
            self._elements.append(item)
            if len(self._elements) > self._queue_len:
                self._elements.popleft()

    def get(self):
        return self._elements.popleft()

    def empty(self):
        if self._elements:
            return False
        return True

    def __getitem__(self, key):
        #with self._mutex:
        item = self._elements[key] #"Deques support thread-safe, memory efficient appends and pops". Lookups also thread safe?
        return item

# A Dict of [Type, Queues]. Each queue entry os unique in the queues
class UniqueQueueDict:
    def __init__(self, queue_len = 10):
        self._elements = deque([])
        self._mutex = Lock()
        self._queue_len = queue_len

    def put(self, item):
        with self._mutex:
            self._elements.append(item)
            if len(self._elements) > self._queue_len:
                self._elements.popleft()

    def get(self):
        return self._elements.popleft()

    def empty(self):
        if self._elements:
            return False
        return True

    def __getitem__(self, key):
        #with self._mutex:
        item = self._elements[key] #"Deques support thread-safe, memory efficient appends and pops". Lookups also thread safe?
        return item