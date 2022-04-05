from multiprocessing import Lock
from collections import deque
from re import I
from typing import Dict, Tuple

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
class UniqueQueueDict():
    def __init__(self, queue_len = 10):
        self._queue_dict = {}
        self._item_dict = {}
        self._queue_len = queue_len
        self._mutex = Lock()

    #add item to queue with key entry. If item is already queued or queue is full, return false
    def add(self, key, item) -> bool:
        with self._mutex:
            if key not in self._queue_dict:
                self._queue_dict[key] = Queue(self._queue_len)

            queue = self._queue_dict[key]
            if len(queue) >= self._queue_len:
                return False

            if item in self._item_dict:
                return False

            queue.put(item)
            self._item_dict[item] = key
            return True

    # Pop item from queue with key in key enty. Delete item
    def pop(self, key):
        with self._mutex:
            if key not in self._queue_dict:
                return None

            val = self._queue_dict[key].get()
            self._item_dict.pop(val)
            return val

    # Return the queue key of the item. If item not queued, return None
    def get_item_queue(self, item):
        with self._mutex:
            if item not in self._item_dict:
                return None
            return self._item_dict[item]

    def is_queue_on_key(self, key):
        if key not in self._queue_dict:
            return False
        if len(self._queue_dict[key]) > 0:
            return True
        return False

    def get_queued_numbers(self) -> Dict[type, int]:
        queue_dict = {}
        for key, queue in self._queue_dict.items():
            queue_dict[key] = len(queue)

    def preview_next_item(self, key):
        queue = self._queue_dict.get(key, None)
        if queue is not None:
            return queue[0]
        return None

    def is_queue_full(self, key):
        with self._mutex:
            if key not in self._queue_dict:
                return False

            queue = self._queue_dict[key]
            return len(queue) >= self._queue_len

    def is_queue_empty(self, key):
        with self._mutex:
            if key not in self._queue_dict:
                return True

            queue = self._queue_dict[key]
            return len(queue) == 0

    def get_queue_len(self, key) -> int:
        with self._mutex:
            if key not in self._queue_dict:
                return 0

            queue = self._queue_dict[key]
            return len(queue)

    def dequeue_item(self, item):
        key = self._item_dict.get(item, None)
        if key is None:
            return

        index = -1
        for i, queue_item in enumerate(self._queue_dict[key]):
            if queue_item == item:
                index = i
        assert index != -1
        del self._queue_dict[key][index]

    def get_queue_item_info(self, item) -> Tuple[type, int]:
        key = self._item_dict.get(item, None)
        if key is None:
            return None, None

        index = -1
        for i, queue_item in enumerate(self._queue_dict[key]):
            if queue_item == item:
                index = i
        assert index != -1
        return key, index