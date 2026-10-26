# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tdt_interface:msg/AutoAimState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'enemy_healths'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AutoAimState(type):
    """Metaclass of message 'AutoAimState'."""

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
            module = import_type_support('tdt_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tdt_interface.msg.AutoAimState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__auto_aim_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__auto_aim_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__auto_aim_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__auto_aim_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__auto_aim_state

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

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


class AutoAimState(metaclass=Metaclass_AutoAimState):
    """Message class 'AutoAimState'."""

    __slots__ = [
        '_header',
        '_frame_sequence',
        '_position',
        '_twist',
        '_yaw_degrees',
        '_pitch_degrees',
        '_health',
        '_max_health',
        '_is_weakened',
        '_projectile_speed',
        '_remaining_seconds',
        '_enemy_healths',
        '_ammo_remaining',
        '_ammo_capacity',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'frame_sequence': 'uint64',
        'position': 'geometry_msgs/Point',
        'twist': 'geometry_msgs/Twist',
        'yaw_degrees': 'float',
        'pitch_degrees': 'float',
        'health': 'int32',
        'max_health': 'int32',
        'is_weakened': 'boolean',
        'projectile_speed': 'float',
        'remaining_seconds': 'float',
        'enemy_healths': 'int32[7]',
        'ammo_remaining': 'int32',
        'ammo_capacity': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 7),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.frame_sequence = kwargs.get('frame_sequence', int())
        from geometry_msgs.msg import Point
        self.position = kwargs.get('position', Point())
        from geometry_msgs.msg import Twist
        self.twist = kwargs.get('twist', Twist())
        self.yaw_degrees = kwargs.get('yaw_degrees', float())
        self.pitch_degrees = kwargs.get('pitch_degrees', float())
        self.health = kwargs.get('health', int())
        self.max_health = kwargs.get('max_health', int())
        self.is_weakened = kwargs.get('is_weakened', bool())
        self.projectile_speed = kwargs.get('projectile_speed', float())
        self.remaining_seconds = kwargs.get('remaining_seconds', float())
        if 'enemy_healths' not in kwargs:
            self.enemy_healths = numpy.zeros(7, dtype=numpy.int32)
        else:
            self.enemy_healths = kwargs.get('enemy_healths')
        self.ammo_remaining = kwargs.get('ammo_remaining', int())
        self.ammo_capacity = kwargs.get('ammo_capacity', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.frame_sequence != other.frame_sequence:
            return False
        if self.position != other.position:
            return False
        if self.twist != other.twist:
            return False
        if self.yaw_degrees != other.yaw_degrees:
            return False
        if self.pitch_degrees != other.pitch_degrees:
            return False
        if self.health != other.health:
            return False
        if self.max_health != other.max_health:
            return False
        if self.is_weakened != other.is_weakened:
            return False
        if self.projectile_speed != other.projectile_speed:
            return False
        if self.remaining_seconds != other.remaining_seconds:
            return False
        if any(self.enemy_healths != other.enemy_healths):
            return False
        if self.ammo_remaining != other.ammo_remaining:
            return False
        if self.ammo_capacity != other.ammo_capacity:
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
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def frame_sequence(self):
        """Message field 'frame_sequence'."""
        return self._frame_sequence

    @frame_sequence.setter
    def frame_sequence(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'frame_sequence' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'frame_sequence' field must be an unsigned integer in [0, 18446744073709551615]"
        self._frame_sequence = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position' field must be a sub message of type 'Point'"
        self._position = value

    @builtins.property
    def twist(self):
        """Message field 'twist'."""
        return self._twist

    @twist.setter
    def twist(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'twist' field must be a sub message of type 'Twist'"
        self._twist = value

    @builtins.property
    def yaw_degrees(self):
        """Message field 'yaw_degrees'."""
        return self._yaw_degrees

    @yaw_degrees.setter
    def yaw_degrees(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'yaw_degrees' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_degrees' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_degrees = value

    @builtins.property
    def pitch_degrees(self):
        """Message field 'pitch_degrees'."""
        return self._pitch_degrees

    @pitch_degrees.setter
    def pitch_degrees(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch_degrees' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_degrees' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_degrees = value

    @builtins.property
    def health(self):
        """Message field 'health'."""
        return self._health

    @health.setter
    def health(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'health' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'health' field must be an integer in [-2147483648, 2147483647]"
        self._health = value

    @builtins.property
    def max_health(self):
        """Message field 'max_health'."""
        return self._max_health

    @max_health.setter
    def max_health(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'max_health' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max_health' field must be an integer in [-2147483648, 2147483647]"
        self._max_health = value

    @builtins.property
    def is_weakened(self):
        """Message field 'is_weakened'."""
        return self._is_weakened

    @is_weakened.setter
    def is_weakened(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'is_weakened' field must be of type 'bool'"
        self._is_weakened = value

    @builtins.property
    def projectile_speed(self):
        """Message field 'projectile_speed'."""
        return self._projectile_speed

    @projectile_speed.setter
    def projectile_speed(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'projectile_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'projectile_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._projectile_speed = value

    @builtins.property
    def remaining_seconds(self):
        """Message field 'remaining_seconds'."""
        return self._remaining_seconds

    @remaining_seconds.setter
    def remaining_seconds(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'remaining_seconds' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'remaining_seconds' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._remaining_seconds = value

    @builtins.property
    def enemy_healths(self):
        """Message field 'enemy_healths'."""
        return self._enemy_healths

    @enemy_healths.setter
    def enemy_healths(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.int32, \
                    "The 'enemy_healths' numpy.ndarray() must have the dtype of 'numpy.int32'"
                assert value.size == 7, \
                    "The 'enemy_healths' numpy.ndarray() must have a size of 7"
                self._enemy_healths = value
                return
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
                 len(value) == 7 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'enemy_healths' field must be a set or sequence with length 7 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._enemy_healths = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def ammo_remaining(self):
        """Message field 'ammo_remaining'."""
        return self._ammo_remaining

    @ammo_remaining.setter
    def ammo_remaining(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'ammo_remaining' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'ammo_remaining' field must be an integer in [-2147483648, 2147483647]"
        self._ammo_remaining = value

    @builtins.property
    def ammo_capacity(self):
        """Message field 'ammo_capacity'."""
        return self._ammo_capacity

    @ammo_capacity.setter
    def ammo_capacity(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'ammo_capacity' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'ammo_capacity' field must be an integer in [-2147483648, 2147483647]"
        self._ammo_capacity = value
