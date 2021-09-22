import enum

import numpy as np


class Side(enum.IntEnum):
    NONE = 0
    LEFT = 1
    RIGHT = 2


class ArrayBuffer(object):
    def __init__(self, dtype=np.float32, init_shape=(2048,)):

        if isinstance(init_shape, int):
            init_shape = (init_shape,)
        self._dtype = dtype
        self._init_shape = init_shape
        self._array = np.zeros(init_shape, dtype)
        self._start = 0
        self._stop = 0
        self._extensions = np.zeros(2048, dtype=[
            ('left', np.int32),
            ('right', np.int32),
            ('lenght', np.int32)
        ])
        self._extension_n = 0

    @property
    def dtype(self):
        return self._dtype

    @property
    def nbytes(self):
        return self._array.nbytes

    @property
    def itemsize(self):
        return self._array.itemsize

    def __len__(self):
        return self._stop - self._start

    def __getitem__(self, key):
        return self._array[self._start:self._stop][key]

    def __setitem__(self, key, value):
        self._array[self._start:self._stop][key] = value

    def _check_enlarge(self, side, extlen):

        if side:
            self._extensions[self._extension_n] = (
                extlen if side == Side.LEFT else 0,
                extlen if side == Side.RIGHT else 0,
                len(self) + extlen
            )
            self._extension_n = ((self._extension_n + 1) %
                                 (len(self._extensions)))

        if side == Side.RIGHT and len(self._array) - self._stop >= extlen:
            return
        elif side == Side.LEFT and self._start >= extlen:
            return

        left_ext = np.sum(self._extensions['left'])
        right_ext = np.sum(self._extensions['right'])
        max_lenght = np.max(self._extensions['lenght'])

        mult = np.int32(np.ceil(max_lenght / max(1, left_ext + right_ext)))

        left_ext *= mult
        right_ext *= mult


        newshape = (
            left_ext + max(len(self), self._init_shape[0]) +
            extlen + right_ext,
            *self._array.shape[1:]
        )

        if side == Side.LEFT:
            newstop = newshape[0] - right_ext
            newstart = newstop - len(self)
        else:
            newstart = left_ext
            newstop = newstart + len(self)

        assert newstop - newstart == len(self)

        newarr = np.zeros(newshape, self._dtype)
        if newstart < newstop:
            newarr[newstart:newstop] = self[:]

        self._start = newstart
        self._stop = newstop
        self._array = newarr

    def extend(self, other):

        ol = len(other)

        self._check_enlarge(Side.RIGHT, ol)
        self._array[self._stop:self._stop + ol] = other[:]
        self._stop += ol

    def extendleft(self, other):

        ol = len(other)

        self._check_enlarge(Side.LEFT, ol)
        self._array[self._start - ol:self._start] = other[:]
        self._start -= ol

    def pop(self, count):

        count = min(count, len(self))

        ret = self[-count:]

        self._stop -= count

        return ret

    def popleft(self, count):

        count = min(count, len(self))

        ret = self[:count]

        self._start += count

        return ret

    def clear(self):

        self.log.debug('Clearing %s', str(self))

        self._start = 0
        self._stop = 0
        self._check_enlarge(Side.NONE, 0)