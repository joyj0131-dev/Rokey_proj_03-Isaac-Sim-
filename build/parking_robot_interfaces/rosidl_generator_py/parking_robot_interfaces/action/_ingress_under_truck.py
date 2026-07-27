# generated from rosidl_generator_py/resource/_idl.py.em
# with input from parking_robot_interfaces:action/IngressUnderTruck.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IngressUnderTruck_Goal(type):
    """Metaclass of message 'IngressUnderTruck_Goal'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IngressUnderTruck_Goal(metaclass=Metaclass_IngressUnderTruck_Goal):
    """Message class 'IngressUnderTruck_Goal'."""

    __slots__ = [
        '_trough_index',
        '_forward_speed',
        '_return_speed',
    ]

    _fields_and_field_types = {
        'trough_index': 'int32',
        'forward_speed': 'float',
        'return_speed': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.trough_index = kwargs.get('trough_index', int())
        self.forward_speed = kwargs.get('forward_speed', float())
        self.return_speed = kwargs.get('return_speed', float())

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
        if self.trough_index != other.trough_index:
            return False
        if self.forward_speed != other.forward_speed:
            return False
        if self.return_speed != other.return_speed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def trough_index(self):
        """Message field 'trough_index'."""
        return self._trough_index

    @trough_index.setter
    def trough_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trough_index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'trough_index' field must be an integer in [-2147483648, 2147483647]"
        self._trough_index = value

    @builtins.property
    def forward_speed(self):
        """Message field 'forward_speed'."""
        return self._forward_speed

    @forward_speed.setter
    def forward_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'forward_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'forward_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._forward_speed = value

    @builtins.property
    def return_speed(self):
        """Message field 'return_speed'."""
        return self._return_speed

    @return_speed.setter
    def return_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'return_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'return_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._return_speed = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_IngressUnderTruck_Result(type):
    """Metaclass of message 'IngressUnderTruck_Result'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IngressUnderTruck_Result(metaclass=Metaclass_IngressUnderTruck_Result):
    """Message class 'IngressUnderTruck_Result'."""

    __slots__ = [
        '_success',
        '_stop_reason',
        '_stop_x',
        '_target_axle_x',
        '_est_max_lateral_dev_m',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'stop_reason': 'string',
        'stop_x': 'float',
        'target_axle_x': 'float',
        'est_max_lateral_dev_m': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.stop_reason = kwargs.get('stop_reason', str())
        self.stop_x = kwargs.get('stop_x', float())
        self.target_axle_x = kwargs.get('target_axle_x', float())
        self.est_max_lateral_dev_m = kwargs.get('est_max_lateral_dev_m', float())

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
        if self.stop_reason != other.stop_reason:
            return False
        if self.stop_x != other.stop_x:
            return False
        if self.target_axle_x != other.target_axle_x:
            return False
        if self.est_max_lateral_dev_m != other.est_max_lateral_dev_m:
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
    def stop_reason(self):
        """Message field 'stop_reason'."""
        return self._stop_reason

    @stop_reason.setter
    def stop_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'stop_reason' field must be of type 'str'"
        self._stop_reason = value

    @builtins.property
    def stop_x(self):
        """Message field 'stop_x'."""
        return self._stop_x

    @stop_x.setter
    def stop_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'stop_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'stop_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._stop_x = value

    @builtins.property
    def target_axle_x(self):
        """Message field 'target_axle_x'."""
        return self._target_axle_x

    @target_axle_x.setter
    def target_axle_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_axle_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_axle_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_axle_x = value

    @builtins.property
    def est_max_lateral_dev_m(self):
        """Message field 'est_max_lateral_dev_m'."""
        return self._est_max_lateral_dev_m

    @est_max_lateral_dev_m.setter
    def est_max_lateral_dev_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_max_lateral_dev_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'est_max_lateral_dev_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._est_max_lateral_dev_m = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_IngressUnderTruck_Feedback(type):
    """Metaclass of message 'IngressUnderTruck_Feedback'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IngressUnderTruck_Feedback(metaclass=Metaclass_IngressUnderTruck_Feedback):
    """Message class 'IngressUnderTruck_Feedback'."""

    __slots__ = [
        '_phase',
        '_current_x',
        '_troughs_seen',
        '_vy_cmd',
    ]

    _fields_and_field_types = {
        'phase': 'string',
        'current_x': 'float',
        'troughs_seen': 'int32',
        'vy_cmd': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.phase = kwargs.get('phase', str())
        self.current_x = kwargs.get('current_x', float())
        self.troughs_seen = kwargs.get('troughs_seen', int())
        self.vy_cmd = kwargs.get('vy_cmd', float())

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
        if self.current_x != other.current_x:
            return False
        if self.troughs_seen != other.troughs_seen:
            return False
        if self.vy_cmd != other.vy_cmd:
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
                isinstance(value, str), \
                "The 'phase' field must be of type 'str'"
        self._phase = value

    @builtins.property
    def current_x(self):
        """Message field 'current_x'."""
        return self._current_x

    @current_x.setter
    def current_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_x = value

    @builtins.property
    def troughs_seen(self):
        """Message field 'troughs_seen'."""
        return self._troughs_seen

    @troughs_seen.setter
    def troughs_seen(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'troughs_seen' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'troughs_seen' field must be an integer in [-2147483648, 2147483647]"
        self._troughs_seen = value

    @builtins.property
    def vy_cmd(self):
        """Message field 'vy_cmd'."""
        return self._vy_cmd

    @vy_cmd.setter
    def vy_cmd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vy_cmd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vy_cmd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vy_cmd = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_IngressUnderTruck_SendGoal_Request(type):
    """Metaclass of message 'IngressUnderTruck_SendGoal_Request'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__send_goal__request

            from parking_robot_interfaces.action import IngressUnderTruck
            if IngressUnderTruck.Goal.__class__._TYPE_SUPPORT is None:
                IngressUnderTruck.Goal.__class__.__import_type_support__()

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


class IngressUnderTruck_SendGoal_Request(metaclass=Metaclass_IngressUnderTruck_SendGoal_Request):
    """Message class 'IngressUnderTruck_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'parking_robot_interfaces/IngressUnderTruck_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['parking_robot_interfaces', 'action'], 'IngressUnderTruck_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Goal
        self.goal = kwargs.get('goal', IngressUnderTruck_Goal())

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
            from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Goal
            assert \
                isinstance(value, IngressUnderTruck_Goal), \
                "The 'goal' field must be a sub message of type 'IngressUnderTruck_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_IngressUnderTruck_SendGoal_Response(type):
    """Metaclass of message 'IngressUnderTruck_SendGoal_Response'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__send_goal__response

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


class IngressUnderTruck_SendGoal_Response(metaclass=Metaclass_IngressUnderTruck_SendGoal_Response):
    """Message class 'IngressUnderTruck_SendGoal_Response'."""

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


class Metaclass_IngressUnderTruck_SendGoal(type):
    """Metaclass of service 'IngressUnderTruck_SendGoal'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__ingress_under_truck__send_goal

            from parking_robot_interfaces.action import _ingress_under_truck
            if _ingress_under_truck.Metaclass_IngressUnderTruck_SendGoal_Request._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_SendGoal_Request.__import_type_support__()
            if _ingress_under_truck.Metaclass_IngressUnderTruck_SendGoal_Response._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_SendGoal_Response.__import_type_support__()


class IngressUnderTruck_SendGoal(metaclass=Metaclass_IngressUnderTruck_SendGoal):
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_SendGoal_Request as Request
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_IngressUnderTruck_GetResult_Request(type):
    """Metaclass of message 'IngressUnderTruck_GetResult_Request'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__get_result__request

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


class IngressUnderTruck_GetResult_Request(metaclass=Metaclass_IngressUnderTruck_GetResult_Request):
    """Message class 'IngressUnderTruck_GetResult_Request'."""

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


class Metaclass_IngressUnderTruck_GetResult_Response(type):
    """Metaclass of message 'IngressUnderTruck_GetResult_Response'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__get_result__response

            from parking_robot_interfaces.action import IngressUnderTruck
            if IngressUnderTruck.Result.__class__._TYPE_SUPPORT is None:
                IngressUnderTruck.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IngressUnderTruck_GetResult_Response(metaclass=Metaclass_IngressUnderTruck_GetResult_Response):
    """Message class 'IngressUnderTruck_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'parking_robot_interfaces/IngressUnderTruck_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['parking_robot_interfaces', 'action'], 'IngressUnderTruck_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Result
        self.result = kwargs.get('result', IngressUnderTruck_Result())

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
            from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Result
            assert \
                isinstance(value, IngressUnderTruck_Result), \
                "The 'result' field must be a sub message of type 'IngressUnderTruck_Result'"
        self._result = value


class Metaclass_IngressUnderTruck_GetResult(type):
    """Metaclass of service 'IngressUnderTruck_GetResult'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__ingress_under_truck__get_result

            from parking_robot_interfaces.action import _ingress_under_truck
            if _ingress_under_truck.Metaclass_IngressUnderTruck_GetResult_Request._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_GetResult_Request.__import_type_support__()
            if _ingress_under_truck.Metaclass_IngressUnderTruck_GetResult_Response._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_GetResult_Response.__import_type_support__()


class IngressUnderTruck_GetResult(metaclass=Metaclass_IngressUnderTruck_GetResult):
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_GetResult_Request as Request
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_IngressUnderTruck_FeedbackMessage(type):
    """Metaclass of message 'IngressUnderTruck_FeedbackMessage'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ingress_under_truck__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ingress_under_truck__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ingress_under_truck__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__ingress_under_truck__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ingress_under_truck__feedback_message

            from parking_robot_interfaces.action import IngressUnderTruck
            if IngressUnderTruck.Feedback.__class__._TYPE_SUPPORT is None:
                IngressUnderTruck.Feedback.__class__.__import_type_support__()

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


class IngressUnderTruck_FeedbackMessage(metaclass=Metaclass_IngressUnderTruck_FeedbackMessage):
    """Message class 'IngressUnderTruck_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'parking_robot_interfaces/IngressUnderTruck_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['parking_robot_interfaces', 'action'], 'IngressUnderTruck_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Feedback
        self.feedback = kwargs.get('feedback', IngressUnderTruck_Feedback())

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
            from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Feedback
            assert \
                isinstance(value, IngressUnderTruck_Feedback), \
                "The 'feedback' field must be a sub message of type 'IngressUnderTruck_Feedback'"
        self._feedback = value


class Metaclass_IngressUnderTruck(type):
    """Metaclass of action 'IngressUnderTruck'."""

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
                'parking_robot_interfaces.action.IngressUnderTruck')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__ingress_under_truck

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from parking_robot_interfaces.action import _ingress_under_truck
            if _ingress_under_truck.Metaclass_IngressUnderTruck_SendGoal._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_SendGoal.__import_type_support__()
            if _ingress_under_truck.Metaclass_IngressUnderTruck_GetResult._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_GetResult.__import_type_support__()
            if _ingress_under_truck.Metaclass_IngressUnderTruck_FeedbackMessage._TYPE_SUPPORT is None:
                _ingress_under_truck.Metaclass_IngressUnderTruck_FeedbackMessage.__import_type_support__()


class IngressUnderTruck(metaclass=Metaclass_IngressUnderTruck):

    # The goal message defined in the action definition.
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Goal as Goal
    # The result message defined in the action definition.
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Result as Result
    # The feedback message defined in the action definition.
    from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from parking_robot_interfaces.action._ingress_under_truck import IngressUnderTruck_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
