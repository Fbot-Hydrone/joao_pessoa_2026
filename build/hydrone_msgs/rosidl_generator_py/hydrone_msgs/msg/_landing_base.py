# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hydrone_msgs:msg/LandingBase.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LandingBase(type):
    """Metaclass of message 'LandingBase'."""

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
            module = import_type_support('hydrone_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hydrone_msgs.msg.LandingBase')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__landing_base
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__landing_base
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__landing_base
            cls._TYPE_SUPPORT = module.type_support_msg__msg__landing_base
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__landing_base

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

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


class LandingBase(metaclass=Metaclass_LandingBase):
    """Message class 'LandingBase'."""

    __slots__ = [
        '_header',
        '_base_id',
        '_pose',
        '_is_suspended',
        '_is_visited',
        '_confidence',
        '_height',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'base_id': 'uint8',
        'pose': 'geometry_msgs/Pose',
        'is_suspended': 'boolean',
        'is_visited': 'boolean',
        'confidence': 'float',
        'height': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.base_id = kwargs.get('base_id', int())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        self.is_suspended = kwargs.get('is_suspended', bool())
        self.is_visited = kwargs.get('is_visited', bool())
        self.confidence = kwargs.get('confidence', float())
        self.height = kwargs.get('height', float())

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
        if self.base_id != other.base_id:
            return False
        if self.pose != other.pose:
            return False
        if self.is_suspended != other.is_suspended:
            return False
        if self.is_visited != other.is_visited:
            return False
        if self.confidence != other.confidence:
            return False
        if self.height != other.height:
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
    def base_id(self):
        """Message field 'base_id'."""
        return self._base_id

    @base_id.setter
    def base_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'base_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'base_id' field must be an unsigned integer in [0, 255]"
        self._base_id = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value

    @builtins.property
    def is_suspended(self):
        """Message field 'is_suspended'."""
        return self._is_suspended

    @is_suspended.setter
    def is_suspended(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_suspended' field must be of type 'bool'"
        self._is_suspended = value

    @builtins.property
    def is_visited(self):
        """Message field 'is_visited'."""
        return self._is_visited

    @is_visited.setter
    def is_visited(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_visited' field must be of type 'bool'"
        self._is_visited = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'height' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._height = value
