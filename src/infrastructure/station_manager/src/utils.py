import copy

class TwoWayDict(dict):
    def __setitem__(self, key, value):
        if key in self:
            del self[key]
        if value in self:
            del self[value]
        dict.__setitem__(self, key, value)
        dict.__setitem__(self, value, key)

    def __delitem__(self, key):
        dict.__delitem__(self, self[key])
        dict.__delitem__(self, key)

    def pop(self, key):
        val = copy.deepcopy(self[key])
        self.__delitem__(key)
        return val

    def __len__(self):
        """Returns the number of connections"""
        return dict.__len__(self) // 2