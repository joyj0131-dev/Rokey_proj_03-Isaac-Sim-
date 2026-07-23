# generated from rosidl_generator_py/resource/_idl.py.em
# with input from parking_robot_interfaces:srv/GetTaskStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTaskStatus_Request(type):
    """Metaclass of message 'GetTaskStatus_Request'."""

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
                'parking_robot_interfaces.srv.GetTaskStatus_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_task_status__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_task_status__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_task_status__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_task_status__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_task_status__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTaskStatus_Request(metaclass=Metaclass_GetTaskStatus_Request):
    """Message class 'GetTaskStatus_Request'."""

    __slots__ = [
        '_task_id',
    ]

    _fields_and_field_types = {
        'task_id': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.task_id = kwargs.get('task_id', str())

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
        if self.task_id != other.task_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTaskStatus_Response(type):
    """Metaclass of message 'GetTaskStatus_Response'."""

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
                'parking_robot_interfaces.srv.GetTaskStatus_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_task_status__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_task_status__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_task_status__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_task_status__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_task_status__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTaskStatus_Response(metaclass=Metaclass_GetTaskStatus_Response):
    """Message class 'GetTaskStatus_Response'."""

    __slots__ = [
        '_state',
        '_eta_seconds',
        '_message',
    ]

    _fields_and_field_types = {
        'state': 'string',
        'eta_seconds': 'int32',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.state = kwargs.get('state', str())
        self.eta_seconds = kwargs.get('eta_seconds', int())
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
        if self.state != other.state:
            return False
        if self.eta_seconds != other.eta_seconds:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'state' field must be of type 'str'"
        self._state = value

    @builtins.property
    def eta_seconds(self):
        """Message field 'eta_seconds'."""
        return self._eta_seconds

    @eta_seconds.setter
    def eta_seconds(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'eta_seconds' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'eta_seconds' field must be an integer in [-2147483648, 2147483647]"
        self._eta_seconds = value

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


class Metaclass_GetTaskStatus(type):
    """Metaclass of service 'GetTaskStatus'."""

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
                'parking_robot_interfaces.srv.GetTaskStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_task_status

            from parking_robot_interfaces.srv import _get_task_status
            if _get_task_status.Metaclass_GetTaskStatus_Request._TYPE_SUPPORT is None:
                _get_task_status.Metaclass_GetTaskStatus_Request.__import_type_support__()
            if _get_task_status.Metaclass_GetTaskStatus_Response._TYPE_SUPPORT is None:
                _get_task_status.Metaclass_GetTaskStatus_Response.__import_type_support__()


class GetTaskStatus(metaclass=Metaclass_GetTaskStatus):
    from parking_robot_interfaces.srv._get_task_status import GetTaskStatus_Request as Request
    from parking_robot_interfaces.srv._get_task_status import GetTaskStatus_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
