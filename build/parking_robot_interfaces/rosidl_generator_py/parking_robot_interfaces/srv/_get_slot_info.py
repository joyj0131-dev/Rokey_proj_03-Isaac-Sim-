# generated from rosidl_generator_py/resource/_idl.py.em
# with input from parking_robot_interfaces:srv/GetSlotInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetSlotInfo_Request(type):
    """Metaclass of message 'GetSlotInfo_Request'."""

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
                'parking_robot_interfaces.srv.GetSlotInfo_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_slot_info__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_slot_info__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_slot_info__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_slot_info__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_slot_info__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSlotInfo_Request(metaclass=Metaclass_GetSlotInfo_Request):
    """Message class 'GetSlotInfo_Request'."""

    __slots__ = [
        '_slot_id',
    ]

    _fields_and_field_types = {
        'slot_id': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.slot_id = kwargs.get('slot_id', str())

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
        if self.slot_id != other.slot_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def slot_id(self):
        """Message field 'slot_id'."""
        return self._slot_id

    @slot_id.setter
    def slot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'slot_id' field must be of type 'str'"
        self._slot_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetSlotInfo_Response(type):
    """Metaclass of message 'GetSlotInfo_Response'."""

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
                'parking_robot_interfaces.srv.GetSlotInfo_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_slot_info__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_slot_info__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_slot_info__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_slot_info__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_slot_info__response

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSlotInfo_Response(metaclass=Metaclass_GetSlotInfo_Response):
    """Message class 'GetSlotInfo_Response'."""

    __slots__ = [
        '_data_ready',
        '_exists',
        '_occupied',
        '_is_accessible',
        '_pose',
    ]

    _fields_and_field_types = {
        'data_ready': 'boolean',
        'exists': 'boolean',
        'occupied': 'boolean',
        'is_accessible': 'boolean',
        'pose': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.data_ready = kwargs.get('data_ready', bool())
        self.exists = kwargs.get('exists', bool())
        self.occupied = kwargs.get('occupied', bool())
        self.is_accessible = kwargs.get('is_accessible', bool())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())

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
        if self.data_ready != other.data_ready:
            return False
        if self.exists != other.exists:
            return False
        if self.occupied != other.occupied:
            return False
        if self.is_accessible != other.is_accessible:
            return False
        if self.pose != other.pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def data_ready(self):
        """Message field 'data_ready'."""
        return self._data_ready

    @data_ready.setter
    def data_ready(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'data_ready' field must be of type 'bool'"
        self._data_ready = value

    @builtins.property
    def exists(self):
        """Message field 'exists'."""
        return self._exists

    @exists.setter
    def exists(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'exists' field must be of type 'bool'"
        self._exists = value

    @builtins.property
    def occupied(self):
        """Message field 'occupied'."""
        return self._occupied

    @occupied.setter
    def occupied(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'occupied' field must be of type 'bool'"
        self._occupied = value

    @builtins.property
    def is_accessible(self):
        """Message field 'is_accessible'."""
        return self._is_accessible

    @is_accessible.setter
    def is_accessible(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_accessible' field must be of type 'bool'"
        self._is_accessible = value

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


class Metaclass_GetSlotInfo(type):
    """Metaclass of service 'GetSlotInfo'."""

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
                'parking_robot_interfaces.srv.GetSlotInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_slot_info

            from parking_robot_interfaces.srv import _get_slot_info
            if _get_slot_info.Metaclass_GetSlotInfo_Request._TYPE_SUPPORT is None:
                _get_slot_info.Metaclass_GetSlotInfo_Request.__import_type_support__()
            if _get_slot_info.Metaclass_GetSlotInfo_Response._TYPE_SUPPORT is None:
                _get_slot_info.Metaclass_GetSlotInfo_Response.__import_type_support__()


class GetSlotInfo(metaclass=Metaclass_GetSlotInfo):
    from parking_robot_interfaces.srv._get_slot_info import GetSlotInfo_Request as Request
    from parking_robot_interfaces.srv._get_slot_info import GetSlotInfo_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
