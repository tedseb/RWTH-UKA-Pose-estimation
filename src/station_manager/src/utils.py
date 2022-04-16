import copy

class TwoWayDict:
    def __init__(self) -> None:
        self._keys = {}
        self._values = {}

    def __setitem__(self, key, value):
        """only here key and value role is importand"""
        if key in self._keys:
            del self._keys[key]
        if value in self._values:
            del self._values[value]
        self._keys.__setitem__(key, value)
        self._values.__setitem__(value, key)

    def __getitem__(self, key):
        value = self.get(key)
        if key is None:
            raise KeyError
        return value

    def __delitem__(self, key):
        if key in self._keys:
            value = self._keys.pop(key)
            del self._values[value]
        elif key in self._values:
            value = self._values.pop(key)
            del self._keys[value]
        else:
            raise KeyError

    def __contains__(self, key) -> bool:
        if key in self._keys:
            return True
        if key in self._values:
            return True
        return False

    def __str__(self):
        return f"[{self._keys}, {self._values}]"

    def pop(self, key):
        val = copy.deepcopy(self.__getitem__(key))
        self.__delitem__(key)
        return val

    def get(self, key, default=None):
        if key in self._keys:
            return self._keys[key]
        if key in self._values:
            return self._values[key]
        return default

    def get_keys(self):
        return self._keys()

    def get_values(self):
        return self._values

    def __len__(self):
        return self._keys.__len__()