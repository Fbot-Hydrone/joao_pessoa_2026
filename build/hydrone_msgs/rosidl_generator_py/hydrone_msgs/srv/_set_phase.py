# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hydrone_msgs:srv/SetPhase.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetPhase_Request(type):
    """Metaclass of message 'SetPhase_Request'."""

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
                'hydrone_msgs.srv.SetPhase_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_phase__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_phase__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_phase__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_phase__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_phase__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetPhase_Request(metaclass=Metaclass_SetPhase_Request):
    """Message class 'SetPhase_Request'."""

    __slots__ = [
        '_phase',
        '_open_hardware',
        '_use_two_drones',
    ]

    _fields_and_field_types = {
        'phase': 'uint8',
        'open_hardware': 'boolean',
        'use_two_drones': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.phase = kwargs.get('phase', int())
        self.open_hardware = kwargs.get('open_hardware', bool())
        self.use_two_drones = kwargs.get('use_two_drones', bool())

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
        if self.phase != other.phase:
            return False
        if self.open_hardware != other.open_hardware:
            return False
        if self.use_two_drones != other.use_two_drones:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def phase(self):
        """Message field 'phase'."""
        return self._phase

    @phase.setter
    def phase(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'phase' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'phase' field must be an unsigned integer in [0, 255]"
        self._phase = value

    @builtins.property
    def open_hardware(self):
        """Message field 'open_hardware'."""
        return self._open_hardware

    @open_hardware.setter
    def open_hardware(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'open_hardware' field must be of type 'bool'"
        self._open_hardware = value

    @builtins.property
    def use_two_drones(self):
        """Message field 'use_two_drones'."""
        return self._use_two_drones

    @use_two_drones.setter
    def use_two_drones(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_two_drones' field must be of type 'bool'"
        self._use_two_drones = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetPhase_Response(type):
    """Metaclass of message 'SetPhase_Response'."""

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
                'hydrone_msgs.srv.SetPhase_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_phase__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_phase__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_phase__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_phase__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_phase__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetPhase_Response(metaclass=Metaclass_SetPhase_Response):
    """Message class 'SetPhase_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SetPhase(type):
    """Metaclass of service 'SetPhase'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hydrone_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hydrone_msgs.srv.SetPhase')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_phase

            from hydrone_msgs.srv import _set_phase
            if _set_phase.Metaclass_SetPhase_Request._TYPE_SUPPORT is None:
                _set_phase.Metaclass_SetPhase_Request.__import_type_support__()
            if _set_phase.Metaclass_SetPhase_Response._TYPE_SUPPORT is None:
                _set_phase.Metaclass_SetPhase_Response.__import_type_support__()


class SetPhase(metaclass=Metaclass_SetPhase):
    from hydrone_msgs.srv._set_phase import SetPhase_Request as Request
    from hydrone_msgs.srv._set_phase import SetPhase_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
