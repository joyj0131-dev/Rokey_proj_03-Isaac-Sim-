# generated from rosidl_generator_py/resource/_idl.py.em
# with input from parking_robot_interfaces:srv/FindEmptySlot.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FindEmptySlot_Request(type):
    """Metaclass of message 'FindEmptySlot_Request'."""

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
                'parking_robot_interfaces.srv.FindEmptySlot_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__find_empty_slot__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__find_empty_slot__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__find_empty_slot__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__find_empty_slot__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__find_empty_slot__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FindEmptySlot_Request(metaclass=Metaclass_FindEmptySlot_Request):
    """Message class 'FindEmptySlot_Request'."""

    __slots__ = [
        '_vehicle_length',
        '_vehicle_width',
    ]

    _fields_and_field_types = {
        'vehicle_length': 'double',
        'vehicle_width': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.vehicle_length = kwargs.get('vehicle_length', float())
        self.vehicle_width = kwargs.get('vehicle_width', float())

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
        if self.vehicle_length != other.vehicle_length:
            return False
        if self.vehicle_width != other.vehicle_width:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vehicle_length(self):
        """Message field 'vehicle_length'."""
        return self._vehicle_length

    @vehicle_length.setter
    def vehicle_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vehicle_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vehicle_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vehicle_length = value

    @builtins.property
    def vehicle_width(self):
        """Message field 'vehicle_width'."""
        return self._vehicle_width

    @vehicle_width.setter
    def vehicle_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vehicle_width' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vehicle_width' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vehicle_width = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_FindEmptySlot_Response(type):
    """Metaclass of message 'FindEmptySlot_Response'."""

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
                'parking_robot_interfaces.srv.FindEmptySlot_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__find_empty_slot__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__find_empty_slot__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__find_empty_slot__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__find_empty_slot__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__find_empty_slot__response

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


class FindEmptySlot_Response(metaclass=Metaclass_FindEmptySlot_Response):
    """Message class 'FindEmptySlot_Response'."""

    __slots__ = [
        '_success',
        '_slot_id',
        '_slot_pose',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'slot_id': 'string',
        'slot_pose': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.slot_id = kwargs.get('slot_id', str())
        from geometry_msgs.msg import Pose
        self.slot_pose = kwargs.get('slot_pose', Pose())

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
        if self.slot_id != other.slot_id:
            return False
        if self.slot_pose != other.slot_pose:
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


class Metaclass_FindEmptySlot(type):
    """Metaclass of service 'FindEmptySlot'."""

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
                'parking_robot_interfaces.srv.FindEmptySlot')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__find_empty_slot

            from parking_robot_interfaces.srv import _find_empty_slot
            if _find_empty_slot.Metaclass_FindEmptySlot_Request._TYPE_SUPPORT is None:
                _find_empty_slot.Metaclass_FindEmptySlot_Request.__import_type_support__()
            if _find_empty_slot.Metaclass_FindEmptySlot_Response._TYPE_SUPPORT is None:
                _find_empty_slot.Metaclass_FindEmptySlot_Response.__import_type_support__()


class FindEmptySlot(metaclass=Metaclass_FindEmptySlot):
    from parking_robot_interfaces.srv._find_empty_slot import FindEmptySlot_Request as Request
    from parking_robot_interfaces.srv._find_empty_slot import FindEmptySlot_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
