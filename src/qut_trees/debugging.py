from enum import Enum


class DebugMode(Enum):
    OFF = 0
    INSTANT_SUCCESS = 1  #  Returns on first tick with SUCCESS
    INPUT_FOR_SUCCESS = 2  #  Returns success when a key press is detected
    INSTANT_FAILURE = 3
    INPUT_FOR_FAILURE = 4

    def invert(self):
        if self == DebugMode.OFF:
            return DebugMode.OFF
        elif self == DebugMode.INSTANT_SUCCESS:
            return DebugMode.INSTANT_FAILURE
        elif self == DebugMode.INPUT_FOR_SUCCESS:
            return DebugMode.INPUT_FOR_FAILURE
        elif self == DebugMode.INSTANT_FAILURE:
            return DebugMode.INSTANT_SUCCESS
        elif self == DebugMode.INPUT_FOR_FAILURE:
            return DebugMode.INPUT_FOR_SUCCESS
