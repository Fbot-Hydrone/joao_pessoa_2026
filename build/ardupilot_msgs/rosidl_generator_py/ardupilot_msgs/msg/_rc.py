# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ardupilot_msgs:msg/Rc.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'channels'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Rc(type):
    """Metaclass of message 'Rc'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ardupilot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ardupilot_msgs.msg.Rc')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rc
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rc
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rc
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rc
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rc

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Rc(metaclass=Metaclass_Rc):
    """Message class 'Rc'."""

    __slots__ = [
        '_header',
        '_is_connected',
        '_receiver_rssi',
        '_channels',
        '_active_overrides',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'is_connected': 'boolean',
        'receiver_rssi': 'uint8',
        'channels': 'sequence<int16, 32>',
        'active_overrides': 'sequence<boolean, 32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('int16'), 32),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('boolean'), 32),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.is_connected = kwargs.get('is_connected', bool())
        self.receiver_rssi = kwargs.get('receiver_rssi', int())
        self.channels = array.array('h', kwargs.get('channels', []))
        self.active_overrides = kwargs.get('active_overrides', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.is_connected != other.is_connected:
            return False
        if self.receiver_rssi != other.receiver_rssi:
            return False
        if self.channels != other.channels:
            return False
        if self.active_overrides != other.active_overrides:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def is_connected(self):
        """Message field 'is_connected'."""
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_connected' field must be of type 'bool'"
        self._is_connected = value

    @builtins.property
    def receiver_rssi(self):
        """Message field 'receiver_rssi'."""
        return self._receiver_rssi

    @receiver_rssi.setter
    def receiver_rssi(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'receiver_rssi' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'receiver_rssi' field must be an unsigned integer in [0, 255]"
        self._receiver_rssi = value

    @builtins.property
    def channels(self):
        """Message field 'channels'."""
        return self._channels

    @channels.setter
    def channels(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'h', \
                "The 'channels' array.array() must have the type code of 'h'"
            assert len(value) <= 32, \
                "The 'channels' array.array() must have a size <= 32"
            self._channels = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 32 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'channels' field must be a set or sequence with length <= 32 and each value of type 'int' and each integer in [-32768, 32767]"
        self._channels = array.array('h', value)

    @builtins.property
    def active_overrides(self):
        """Message field 'active_overrides'."""
        return self._active_overrides

    @active_overrides.setter
    def active_overrides(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 32 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'active_overrides' field must be a set or sequence with length <= 32 and each value of type 'bool'"
        self._active_overrides = value
