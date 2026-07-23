# generated from rosidl_generator_py/resource/_idl.py.em
# with input from parking_robot_interfaces:action/ExecuteParkingTask.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExecuteParkingTask_Goal(type):
    """Metaclass of message 'ExecuteParkingTask_Goal'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__goal

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


class ExecuteParkingTask_Goal(metaclass=Metaclass_ExecuteParkingTask_Goal):
    """Message class 'ExecuteParkingTask_Goal'."""

    __slots__ = [
        '_task_id',
        '_request_type',
        '_vehicle_id',
        '_slot_id',
        '_slot_pose',
        '_leader_robot_id',
        '_follower_robot_id',
    ]

    _fields_and_field_types = {
        'task_id': 'string',
        'request_type': 'string',
        'vehicle_id': 'string',
        'slot_id': 'string',
        'slot_pose': 'geometry_msgs/Pose',
        'leader_robot_id': 'string',
        'follower_robot_id': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.task_id = kwargs.get('task_id', str())
        self.request_type = kwargs.get('request_type', str())
        self.vehicle_id = kwargs.get('vehicle_id', str())
        self.slot_id = kwargs.get('slot_id', str())
        from geometry_msgs.msg import Pose
        self.slot_pose = kwargs.get('slot_pose', Pose())
        self.leader_robot_id = kwargs.get('leader_robot_id', str())
        self.follower_robot_id = kwargs.get('follower_robot_id', str())

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
        if self.request_type != other.request_type:
            return False
        if self.vehicle_id != other.vehicle_id:
            return False
        if self.slot_id != other.slot_id:
            return False
        if self.slot_pose != other.slot_pose:
            return False
        if self.leader_robot_id != other.leader_robot_id:
            return False
        if self.follower_robot_id != other.follower_robot_id:
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

    @builtins.property
    def request_type(self):
        """Message field 'request_type'."""
        return self._request_type

    @request_type.setter
    def request_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'request_type' field must be of type 'str'"
        self._request_type = value

    @builtins.property
    def vehicle_id(self):
        """Message field 'vehicle_id'."""
        return self._vehicle_id

    @vehicle_id.setter
    def vehicle_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'vehicle_id' field must be of type 'str'"
        self._vehicle_id = value

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

    @builtins.property
    def slot_pose(self):
        """Message field 'slot_pose'."""
        return self._slot_pose

    @slot_pose.setter
    def slot_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'slot_pose' field must be a sub message of type 'Pose'"
        self._slot_pose = value

    @builtins.property
    def leader_robot_id(self):
        """Message field 'leader_robot_id'."""
        return self._leader_robot_id

    @leader_robot_id.setter
    def leader_robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'leader_robot_id' field must be of type 'str'"
        self._leader_robot_id = value

    @builtins.property
    def follower_robot_id(self):
        """Message field 'follower_robot_id'."""
        return self._follower_robot_id

    @follower_robot_id.setter
    def follower_robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'follower_robot_id' field must be of type 'str'"
        self._follower_robot_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_Result(type):
    """Metaclass of message 'ExecuteParkingTask_Result'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_Result(metaclass=Metaclass_ExecuteParkingTask_Result):
    """Message class 'ExecuteParkingTask_Result'."""

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


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_Feedback(type):
    """Metaclass of message 'ExecuteParkingTask_Feedback'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_Feedback(metaclass=Metaclass_ExecuteParkingTask_Feedback):
    """Message class 'ExecuteParkingTask_Feedback'."""

    __slots__ = [
        '_current_step',
        '_progress',
    ]

    _fields_and_field_types = {
        'current_step': 'string',
        'progress': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_step = kwargs.get('current_step', str())
        self.progress = kwargs.get('progress', float())

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
        if self.current_step != other.current_step:
            return False
        if self.progress != other.progress:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_step(self):
        """Message field 'current_step'."""
        return self._current_step

    @current_step.setter
    def current_step(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'current_step' field must be of type 'str'"
        self._current_step = value

    @builtins.property
    def progress(self):
        """Message field 'progress'."""
        return self._progress

    @progress.setter
    def progress(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'progress' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'progress' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._progress = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_SendGoal_Request(type):
    """Metaclass of message 'ExecuteParkingTask_SendGoal_Request'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__send_goal__request

            from parking_robot_interfaces.action import ExecuteParkingTask
            if ExecuteParkingTask.Goal.__class__._TYPE_SUPPORT is None:
                ExecuteParkingTask.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_SendGoal_Request(metaclass=Metaclass_ExecuteParkingTask_SendGoal_Request):
    """Message class 'ExecuteParkingTask_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'parking_robot_interfaces/ExecuteParkingTask_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['parking_robot_interfaces', 'action'], 'ExecuteParkingTask_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Goal
        self.goal = kwargs.get('goal', ExecuteParkingTask_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Goal
            assert \
                isinstance(value, ExecuteParkingTask_Goal), \
                "The 'goal' field must be a sub message of type 'ExecuteParkingTask_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_SendGoal_Response(type):
    """Metaclass of message 'ExecuteParkingTask_SendGoal_Response'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_SendGoal_Response(metaclass=Metaclass_ExecuteParkingTask_SendGoal_Response):
    """Message class 'ExecuteParkingTask_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_ExecuteParkingTask_SendGoal(type):
    """Metaclass of service 'ExecuteParkingTask_SendGoal'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__execute_parking_task__send_goal

            from parking_robot_interfaces.action import _execute_parking_task
            if _execute_parking_task.Metaclass_ExecuteParkingTask_SendGoal_Request._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_SendGoal_Request.__import_type_support__()
            if _execute_parking_task.Metaclass_ExecuteParkingTask_SendGoal_Response._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_SendGoal_Response.__import_type_support__()


class ExecuteParkingTask_SendGoal(metaclass=Metaclass_ExecuteParkingTask_SendGoal):
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_SendGoal_Request as Request
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_GetResult_Request(type):
    """Metaclass of message 'ExecuteParkingTask_GetResult_Request'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_GetResult_Request(metaclass=Metaclass_ExecuteParkingTask_GetResult_Request):
    """Message class 'ExecuteParkingTask_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_GetResult_Response(type):
    """Metaclass of message 'ExecuteParkingTask_GetResult_Response'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__get_result__response

            from parking_robot_interfaces.action import ExecuteParkingTask
            if ExecuteParkingTask.Result.__class__._TYPE_SUPPORT is None:
                ExecuteParkingTask.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_GetResult_Response(metaclass=Metaclass_ExecuteParkingTask_GetResult_Response):
    """Message class 'ExecuteParkingTask_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'parking_robot_interfaces/ExecuteParkingTask_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['parking_robot_interfaces', 'action'], 'ExecuteParkingTask_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Result
        self.result = kwargs.get('result', ExecuteParkingTask_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Result
            assert \
                isinstance(value, ExecuteParkingTask_Result), \
                "The 'result' field must be a sub message of type 'ExecuteParkingTask_Result'"
        self._result = value


class Metaclass_ExecuteParkingTask_GetResult(type):
    """Metaclass of service 'ExecuteParkingTask_GetResult'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__execute_parking_task__get_result

            from parking_robot_interfaces.action import _execute_parking_task
            if _execute_parking_task.Metaclass_ExecuteParkingTask_GetResult_Request._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_GetResult_Request.__import_type_support__()
            if _execute_parking_task.Metaclass_ExecuteParkingTask_GetResult_Response._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_GetResult_Response.__import_type_support__()


class ExecuteParkingTask_GetResult(metaclass=Metaclass_ExecuteParkingTask_GetResult):
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_GetResult_Request as Request
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteParkingTask_FeedbackMessage(type):
    """Metaclass of message 'ExecuteParkingTask_FeedbackMessage'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_parking_task__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_parking_task__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_parking_task__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_parking_task__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_parking_task__feedback_message

            from parking_robot_interfaces.action import ExecuteParkingTask
            if ExecuteParkingTask.Feedback.__class__._TYPE_SUPPORT is None:
                ExecuteParkingTask.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteParkingTask_FeedbackMessage(metaclass=Metaclass_ExecuteParkingTask_FeedbackMessage):
    """Message class 'ExecuteParkingTask_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'parking_robot_interfaces/ExecuteParkingTask_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['parking_robot_interfaces', 'action'], 'ExecuteParkingTask_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Feedback
        self.feedback = kwargs.get('feedback', ExecuteParkingTask_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Feedback
            assert \
                isinstance(value, ExecuteParkingTask_Feedback), \
                "The 'feedback' field must be a sub message of type 'ExecuteParkingTask_Feedback'"
        self._feedback = value


class Metaclass_ExecuteParkingTask(type):
    """Metaclass of action 'ExecuteParkingTask'."""

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
                'parking_robot_interfaces.action.ExecuteParkingTask')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__execute_parking_task

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from parking_robot_interfaces.action import _execute_parking_task
            if _execute_parking_task.Metaclass_ExecuteParkingTask_SendGoal._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_SendGoal.__import_type_support__()
            if _execute_parking_task.Metaclass_ExecuteParkingTask_GetResult._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_GetResult.__import_type_support__()
            if _execute_parking_task.Metaclass_ExecuteParkingTask_FeedbackMessage._TYPE_SUPPORT is None:
                _execute_parking_task.Metaclass_ExecuteParkingTask_FeedbackMessage.__import_type_support__()


class ExecuteParkingTask(metaclass=Metaclass_ExecuteParkingTask):

    # The goal message defined in the action definition.
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Goal as Goal
    # The result message defined in the action definition.
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Result as Result
    # The feedback message defined in the action definition.
    from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from parking_robot_interfaces.action._execute_parking_task import ExecuteParkingTask_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
