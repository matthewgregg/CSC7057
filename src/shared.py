class Running(object):
    def __init__(self):
        self._value = False

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        if not isinstance(value, bool):
            raise TypeError
        self._value = value

    @value.deleter
    def value(self):
        del self._value

class Filter(object):
    def __init__(self):
        self._value = None

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value

    @value.deleter
    def value(self):
        del self._value

running = Running()
waiting = Running()
filter_type = Filter()