from pydrake.systems._lcm_py import *

from pydrake.systems.framework import AbstractValue, Value


class PySerializer(SerializerInterface):
    """
    Provides a Python implementation of `SerializerInterface` for use
    with `LcmPublisherSystem` and `LcmSubscriberSystem`.
    """
    def __init__(self, lcm_type):
        SerializerInterface.__init__(self)
        self._lcm_type = lcm_type

    def CreateDefaultValue(self):
        return AbstractValue.Make(self._lcm_type())

    def Deserialize(self, buffer, value):
        message = self._lcm_type.decode(buffer)
        value.set_value(message)

    def Serialize(self, value):
        assert isinstance(value, AbstractValue)
        message = value.get_value()
        assert isinstance(message, self._lcm_type)
        return message.encode()


# Attach methods (as descriptors).
@classmethod
def _make_lcm_system(cls, channel, lcm_type, lcm):
    """
    Convenience to create an LCM system with a concrete type.
    @param channel LCM channel name.
    @param lcm_type LCM type.
    @param lcm LCM service instance.
    """
    serializer = PySerializer(lcm_type)
    return cls(channel, serializer, lcm)


LcmPublisherSystem.Make = _make_lcm_system
LcmSubscriberSystem.Make = _make_lcm_system
