class Running(object):
    def __init__(self):
        self._value = False

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, bool)
        self._value = value

    @value.deleter
    def value(self):
        del self._value


running = Running()
waiting = Running()
