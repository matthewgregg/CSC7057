class Running(object):
    """Running object to store the state of the application"""
    def __init__(self):
        """
        Instantiate Running object. Initial angle is set to False
        """
        self._value = False

    @property
    def value(self) -> bool:
        return self._value

    @value.setter
    def value(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError
        self._value = value

    @value.deleter
    def value(self):
        del self._value


class Filter(object):
    """Filter object to store the selected image filter"""
    def __init__(self):
        """
        Initialise the filter with no selection
        """
        self._value = None

    @property
    def value(self) -> str:
        return self._value

    @value.setter
    def value(self, value: str):
        self._value = value

    @value.deleter
    def value(self):
        del self._value

running = Running()
waiting = Running()
filter_type = Filter()