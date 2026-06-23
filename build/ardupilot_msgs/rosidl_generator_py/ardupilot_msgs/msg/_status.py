# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ardupilot_msgs:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'failsafe'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'APM_ROVER': 1,
        'APM_ARDUCOPTER': 2,
        'APM_ARDUPLANE': 3,
        'APM_ANTENNATRACKER': 4,
        'APM_UNKNOWN': 5,
        'APM_REPLAY': 6,
        'APM_ARDUSUB': 7,
        'APM_IOFIRMWARE': 8,
        'APM_AP_PERIPH': 9,
        'APM_AP_DAL_STANDALONE': 10,
        'APM_AP_BOOTLOADER': 11,
        'APM_BLIMP': 12,
        'APM_HELI': 13,
        'FS_RADIO': 21,
        'FS_BATTERY': 22,
        'FS_GCS': 23,
        'FS_EKF': 24,
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
                'ardupilot_msgs.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'APM_ROVER': cls.__constants['APM_ROVER'],
            'APM_ARDUCOPTER': cls.__constants['APM_ARDUCOPTER'],
            'APM_ARDUPLANE': cls.__constants['APM_ARDUPLANE'],
            'APM_ANTENNATRACKER': cls.__constants['APM_ANTENNATRACKER'],
            'APM_UNKNOWN': cls.__constants['APM_UNKNOWN'],
            'APM_REPLAY': cls.__constants['APM_REPLAY'],
            'APM_ARDUSUB': cls.__constants['APM_ARDUSUB'],
            'APM_IOFIRMWARE': cls.__constants['APM_IOFIRMWARE'],
            'APM_AP_PERIPH': cls.__constants['APM_AP_PERIPH'],
            'APM_AP_DAL_STANDALONE': cls.__constants['APM_AP_DAL_STANDALONE'],
            'APM_AP_BOOTLOADER': cls.__constants['APM_AP_BOOTLOADER'],
            'APM_BLIMP': cls.__constants['APM_BLIMP'],
            'APM_HELI': cls.__constants['APM_HELI'],
            'FS_RADIO': cls.__constants['FS_RADIO'],
            'FS_BATTERY': cls.__constants['FS_BATTERY'],
            'FS_GCS': cls.__constants['FS_GCS'],
            'FS_EKF': cls.__constants['FS_EKF'],
        }

    @property
    def APM_ROVER(self):
        """Message constant 'APM_ROVER'."""
        return Metaclass_Status.__constants['APM_ROVER']

    @property
    def APM_ARDUCOPTER(self):
        """Message constant 'APM_ARDUCOPTER'."""
        return Metaclass_Status.__constants['APM_ARDUCOPTER']

    @property
    def APM_ARDUPLANE(self):
        """Message constant 'APM_ARDUPLANE'."""
        return Metaclass_Status.__constants['APM_ARDUPLANE']

    @property
    def APM_ANTENNATRACKER(self):
        """Message constant 'APM_ANTENNATRACKER'."""
        return Metaclass_Status.__constants['APM_ANTENNATRACKER']

    @property
    def APM_UNKNOWN(self):
        """Message constant 'APM_UNKNOWN'."""
        return Metaclass_Status.__constants['APM_UNKNOWN']

    @property
    def APM_REPLAY(self):
        """Message constant 'APM_REPLAY'."""
        return Metaclass_Status.__constants['APM_REPLAY']

    @property
    def APM_ARDUSUB(self):
        """Message constant 'APM_ARDUSUB'."""
        return Metaclass_Status.__constants['APM_ARDUSUB']

    @property
    def APM_IOFIRMWARE(self):
        """Message constant 'APM_IOFIRMWARE'."""
        return Metaclass_Status.__constants['APM_IOFIRMWARE']

    @property
    def APM_AP_PERIPH(self):
        """Message constant 'APM_AP_PERIPH'."""
        return Metaclass_Status.__constants['APM_AP_PERIPH']

    @property
    def APM_AP_DAL_STANDALONE(self):
        """Message constant 'APM_AP_DAL_STANDALONE'."""
        return Metaclass_Status.__constants['APM_AP_DAL_STANDALONE']

    @property
    def APM_AP_BOOTLOADER(self):
        """Message constant 'APM_AP_BOOTLOADER'."""
        return Metaclass_Status.__constants['APM_AP_BOOTLOADER']

    @property
    def APM_BLIMP(self):
        """Message constant 'APM_BLIMP'."""
        return Metaclass_Status.__constants['APM_BLIMP']

    @property
    def APM_HELI(self):
        """Message constant 'APM_HELI'."""
        return Metaclass_Status.__constants['APM_HELI']

    @property
    def FS_RADIO(self):
        """Message constant 'FS_RADIO'."""
        return Metaclass_Status.__constants['FS_RADIO']

    @property
    def FS_BATTERY(self):
        """Message constant 'FS_BATTERY'."""
        return Metaclass_Status.__constants['FS_BATTERY']

    @property
    def FS_GCS(self):
        """Message constant 'FS_GCS'."""
        return Metaclass_Status.__constants['FS_GCS']

    @property
    def FS_EKF(self):
        """Message constant 'FS_EKF'."""
        return Metaclass_Status.__constants['FS_EKF']


class Status(metaclass=Metaclass_Status):
    """
    Message class 'Status'.

    Constants:
      APM_ROVER
      APM_ARDUCOPTER
      APM_ARDUPLANE
      APM_ANTENNATRACKER
      APM_UNKNOWN
      APM_REPLAY
      APM_ARDUSUB
      APM_IOFIRMWARE
      APM_AP_PERIPH
      APM_AP_DAL_STANDALONE
      APM_AP_BOOTLOADER
      APM_BLIMP
      APM_HELI
      FS_RADIO
      FS_BATTERY
      FS_GCS
      FS_EKF
    """

    __slots__ = [
        '_header',
        '_vehicle_type',
        '_armed',
        '_mode',
        '_flying',
        '_external_control',
        '_failsafe',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'vehicle_type': 'uint8',
        'armed': 'boolean',
        'mode': 'uint8',
        'flying': 'boolean',
        'external_control': 'boolean',
        'failsafe': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.vehicle_type = kwargs.get('vehicle_type', int())
        self.armed = kwargs.get('armed', bool())
        self.mode = kwargs.get('mode', int())
        self.flying = kwargs.get('flying', bool())
        self.external_control = kwargs.get('external_control', bool())
        self.failsafe = array.array('B', kwargs.get('failsafe', []))

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
        if self.vehicle_type != other.vehicle_type:
            return False
        if self.armed != other.armed:
            return False
        if self.mode != other.mode:
            return False
        if self.flying != other.flying:
            return False
        if self.external_control != other.external_control:
            return False
        if self.failsafe != other.failsafe:
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
    def vehicle_type(self):
        """Message field 'vehicle_type'."""
        return self._vehicle_type

    @vehicle_type.setter
    def vehicle_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vehicle_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'vehicle_type' field must be an unsigned integer in [0, 255]"
        self._vehicle_type = value

    @builtins.property
    def armed(self):
        """Message field 'armed'."""
        return self._armed

    @armed.setter
    def armed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'armed' field must be of type 'bool'"
        self._armed = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def flying(self):
        """Message field 'flying'."""
        return self._flying

    @flying.setter
    def flying(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'flying' field must be of type 'bool'"
        self._flying = value

    @builtins.property
    def external_control(self):
        """Message field 'external_control'."""
        return self._external_control

    @external_control.setter
    def external_control(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'external_control' field must be of type 'bool'"
        self._external_control = value

    @builtins.property
    def failsafe(self):
        """Message field 'failsafe'."""
        return self._failsafe

    @failsafe.setter
    def failsafe(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'failsafe' array.array() must have the type code of 'B'"
            self._failsafe = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'failsafe' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._failsafe = array.array('B', value)
