# generated from rosidl_generator_py/resource/_idl.py.em
# with input from biguasim_interfaces:msg/ImagingSonar.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ImagingSonar(type):
    """Metaclass of message 'ImagingSonar'."""

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
            module = import_type_support('biguasim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'biguasim_interfaces.msg.ImagingSonar')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imaging_sonar
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imaging_sonar
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imaging_sonar
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imaging_sonar
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imaging_sonar

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

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


class ImagingSonar(metaclass=Metaclass_ImagingSonar):
    """Message class 'ImagingSonar'."""

    __slots__ = [
        '_header',
        '_bins_azimuth',
        '_bins_range',
        '_raw_image',
        '_intensity',
        '_elevation',
        '_point_cloud',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'bins_azimuth': 'int32',
        'bins_range': 'int32',
        'raw_image': 'sensor_msgs/Image',
        'intensity': 'sensor_msgs/Image',
        'elevation': 'sensor_msgs/Image',
        'point_cloud': 'sensor_msgs/PointCloud2',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.bins_azimuth = kwargs.get('bins_azimuth', int())
        self.bins_range = kwargs.get('bins_range', int())
        from sensor_msgs.msg import Image
        self.raw_image = kwargs.get('raw_image', Image())
        from sensor_msgs.msg import Image
        self.intensity = kwargs.get('intensity', Image())
        from sensor_msgs.msg import Image
        self.elevation = kwargs.get('elevation', Image())
        from sensor_msgs.msg import PointCloud2
        self.point_cloud = kwargs.get('point_cloud', PointCloud2())

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
        if self.bins_azimuth != other.bins_azimuth:
            return False
        if self.bins_range != other.bins_range:
            return False
        if self.raw_image != other.raw_image:
            return False
        if self.intensity != other.intensity:
            return False
        if self.elevation != other.elevation:
            return False
        if self.point_cloud != other.point_cloud:
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
    def bins_azimuth(self):
        """Message field 'bins_azimuth'."""
        return self._bins_azimuth

    @bins_azimuth.setter
    def bins_azimuth(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bins_azimuth' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bins_azimuth' field must be an integer in [-2147483648, 2147483647]"
        self._bins_azimuth = value

    @builtins.property
    def bins_range(self):
        """Message field 'bins_range'."""
        return self._bins_range

    @bins_range.setter
    def bins_range(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bins_range' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bins_range' field must be an integer in [-2147483648, 2147483647]"
        self._bins_range = value

    @builtins.property
    def raw_image(self):
        """Message field 'raw_image'."""
        return self._raw_image

    @raw_image.setter
    def raw_image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'raw_image' field must be a sub message of type 'Image'"
        self._raw_image = value

    @builtins.property
    def intensity(self):
        """Message field 'intensity'."""
        return self._intensity

    @intensity.setter
    def intensity(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'intensity' field must be a sub message of type 'Image'"
        self._intensity = value

    @builtins.property
    def elevation(self):
        """Message field 'elevation'."""
        return self._elevation

    @elevation.setter
    def elevation(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'elevation' field must be a sub message of type 'Image'"
        self._elevation = value

    @builtins.property
    def point_cloud(self):
        """Message field 'point_cloud'."""
        return self._point_cloud

    @point_cloud.setter
    def point_cloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'point_cloud' field must be a sub message of type 'PointCloud2'"
        self._point_cloud = value
