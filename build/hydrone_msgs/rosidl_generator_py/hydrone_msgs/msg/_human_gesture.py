# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hydrone_msgs:msg/HumanGesture.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'skeleton_keypoints'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HumanGesture(type):
    """Metaclass of message 'HumanGesture'."""

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
                'hydrone_msgs.msg.HumanGesture')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__human_gesture
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__human_gesture
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__human_gesture
            cls._TYPE_SUPPORT = module.type_support_msg__msg__human_gesture
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__human_gesture

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

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


class HumanGesture(metaclass=Metaclass_HumanGesture):
    """Message class 'HumanGesture'."""

    __slots__ = [
        '_header',
        '_gesture_name',
        '_confidence',
        '_human_position',
        '_skeleton_keypoints',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'gesture_name': 'string',
        'confidence': 'float',
        'human_position': 'geometry_msgs/Point',
        'skeleton_keypoints': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.gesture_name = kwargs.get('gesture_name', str())
        self.confidence = kwargs.get('confidence', float())
        from geometry_msgs.msg import Point
        self.human_position = kwargs.get('human_position', Point())
        self.skeleton_keypoints = array.array('f', kwargs.get('skeleton_keypoints', []))

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
        if self.gesture_name != other.gesture_name:
            return False
        if self.confidence != other.confidence:
            return False
        if self.human_position != other.human_position:
            return False
        if self.skeleton_keypoints != other.skeleton_keypoints:
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
    def gesture_name(self):
        """Message field 'gesture_name'."""
        return self._gesture_name

    @gesture_name.setter
    def gesture_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'gesture_name' field must be of type 'str'"
        self._gesture_name = value

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
    def human_position(self):
        """Message field 'human_position'."""
        return self._human_position

    @human_position.setter
    def human_position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'human_position' field must be a sub message of type 'Point'"
        self._human_position = value

    @builtins.property
    def skeleton_keypoints(self):
        """Message field 'skeleton_keypoints'."""
        return self._skeleton_keypoints

    @skeleton_keypoints.setter
    def skeleton_keypoints(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'skeleton_keypoints' array.array() must have the type code of 'f'"
            self._skeleton_keypoints = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'skeleton_keypoints' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._skeleton_keypoints = array.array('f', value)
