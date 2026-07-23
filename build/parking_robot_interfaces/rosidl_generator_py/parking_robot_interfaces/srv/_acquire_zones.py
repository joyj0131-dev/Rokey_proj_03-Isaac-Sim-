# generated from rosidl_generator_py/resource/_idl.py.em
# with input from parking_robot_interfaces:srv/AcquireZones.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AcquireZones_Request(type):
    """Metaclass of message 'AcquireZones_Request'."""

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
            module = import_type_support('parking_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'parking_robot_interfaces.srv.AcquireZones_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__acquire_zones__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__acquire_zones__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__acquire_zones__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__acquire_zones__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__acquire_zones__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AcquireZones_Request(metaclass=Metaclass_AcquireZones_Request):
    """Message class 'AcquireZones_Request'."""

    __slots__ = [
        '_robot_id',
        '_task_id',
        '_zone_ids',
    ]

    _fields_and_field_types = {
        'robot_id': 'string',
        'task_id': 'string',
        'zone_ids': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', str())
        self.task_id = kwargs.get('task_id', str())
        self.zone_ids = kwargs.get('zone_ids', [])

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
        if self.robot_id != other.robot_id:
            return False
        if self.task_id != other.task_id:
            return False
        if self.zone_ids != other.zone_ids:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_id' field must be of type 'str'"
        self._robot_id = value

    @builtins.property
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_id' field must be of type 'str'"
        self._task_id = value

    @builtins.property
    def zone_ids(self):
        """Message field 'zone_ids'."""
        return self._zone_ids

    @zone_ids.setter
    def zone_ids(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'zone_ids' field must be a set or sequence and each value of type 'str'"
        self._zone_ids = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_AcquireZones_Response(type):
    """Metaclass of message 'AcquireZones_Response'."""

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
            module = import_type_support('parking_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'parking_robot_interfaces.srv.AcquireZones_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__acquire_zones__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__acquire_zones__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__acquire_zones__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__acquire_zones__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__acquire_zones__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AcquireZones_Response(metaclass=Metaclass_AcquireZones_Response):
    """Message class 'AcquireZones_Response'."""

    __slots__ = [
        '_granted',
        '_held_zones',
        '_retry_after_sec',
    ]

    _fields_and_field_types = {
        'granted': 'boolean',
        'held_zones': 'sequence<string>',
        'retry_after_sec': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.granted = kwargs.get('granted', bool())
        self.held_zones = kwargs.get('held_zones', [])
        self.retry_after_sec = kwargs.get('retry_after_sec', float())

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
        if self.granted != other.granted:
            return False
        if self.held_zones != other.held_zones:
            return False
        if self.retry_after_sec != other.retry_after_sec:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def granted(self):
        """Message field 'granted'."""
        return self._granted

    @granted.setter
    def granted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'granted' field must be of type 'bool'"
        self._granted = value

    @builtins.property
    def held_zones(self):
        """Message field 'held_zones'."""
        return self._held_zones

    @held_zones.setter
    def held_zones(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'held_zones' field must be a set or sequence and each value of type 'str'"
        self._held_zones = value

    @builtins.property
    def retry_after_sec(self):
        """Message field 'retry_after_sec'."""
        return self._retry_after_sec

    @retry_after_sec.setter
    def retry_after_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'retry_after_sec' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'retry_after_sec' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._retry_after_sec = value


class Metaclass_AcquireZones(type):
    """Metaclass of service 'AcquireZones'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('parking_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'parking_robot_interfaces.srv.AcquireZones')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__acquire_zones

            from parking_robot_interfaces.srv import _acquire_zones
            if _acquire_zones.Metaclass_AcquireZones_Request._TYPE_SUPPORT is None:
                _acquire_zones.Metaclass_AcquireZones_Request.__import_type_support__()
            if _acquire_zones.Metaclass_AcquireZones_Response._TYPE_SUPPORT is None:
                _acquire_zones.Metaclass_AcquireZones_Response.__import_type_support__()


class AcquireZones(metaclass=Metaclass_AcquireZones):
    from parking_robot_interfaces.srv._acquire_zones import AcquireZones_Request as Request
    from parking_robot_interfaces.srv._acquire_zones import AcquireZones_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
