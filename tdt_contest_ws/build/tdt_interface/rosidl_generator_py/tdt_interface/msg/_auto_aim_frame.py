# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tdt_interface:msg/AutoAimFrame.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AutoAimFrame(type):
    """Metaclass of message 'AutoAimFrame'."""

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
                'tdt_interface.msg.AutoAimFrame')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__auto_aim_frame
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__auto_aim_frame
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__auto_aim_frame
            cls._TYPE_SUPPORT = module.type_support_msg__msg__auto_aim_frame
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__auto_aim_frame

            from sensor_msgs.msg import CameraInfo
            if CameraInfo.__class__._TYPE_SUPPORT is None:
                CameraInfo.__class__.__import_type_support__()

            from sensor_msgs.msg import CompressedImage
            if CompressedImage.__class__._TYPE_SUPPORT is None:
                CompressedImage.__class__.__import_type_support__()

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from tdt_interface.msg import AutoAimState
            if AutoAimState.__class__._TYPE_SUPPORT is None:
                AutoAimState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AutoAimFrame(metaclass=Metaclass_AutoAimFrame):
    """Message class 'AutoAimFrame'."""

    __slots__ = [
        '_header',
        '_state',
        '_camera_info',
        '_image',
        '_compressed_image',
        '_capture_to_publish_ms',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'tdt_interface/AutoAimState',
        'camera_info': 'sensor_msgs/CameraInfo',
        'image': 'sensor_msgs/Image',
        'compressed_image': 'sensor_msgs/CompressedImage',
        'capture_to_publish_ms': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['tdt_interface', 'msg'], 'AutoAimState'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'CameraInfo'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'CompressedImage'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        from tdt_interface.msg import AutoAimState
        self.state = kwargs.get('state', AutoAimState())
        from sensor_msgs.msg import CameraInfo
        self.camera_info = kwargs.get('camera_info', CameraInfo())
        from sensor_msgs.msg import Image
        self.image = kwargs.get('image', Image())
        from sensor_msgs.msg import CompressedImage
        self.compressed_image = kwargs.get('compressed_image', CompressedImage())
        self.capture_to_publish_ms = kwargs.get('capture_to_publish_ms', float())

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
        if self.state != other.state:
            return False
        if self.camera_info != other.camera_info:
            return False
        if self.image != other.image:
            return False
        if self.compressed_image != other.compressed_image:
            return False
        if self.capture_to_publish_ms != other.capture_to_publish_ms:
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
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if self._check_fields:
            from tdt_interface.msg import AutoAimState
            assert \
                isinstance(value, AutoAimState), \
                "The 'state' field must be a sub message of type 'AutoAimState'"
        self._state = value

    @builtins.property
    def camera_info(self):
        """Message field 'camera_info'."""
        return self._camera_info

    @camera_info.setter
    def camera_info(self, value):
        if self._check_fields:
            from sensor_msgs.msg import CameraInfo
            assert \
                isinstance(value, CameraInfo), \
                "The 'camera_info' field must be a sub message of type 'CameraInfo'"
        self._camera_info = value

    @builtins.property
    def image(self):
        """Message field 'image'."""
        return self._image

    @image.setter
    def image(self, value):
        if self._check_fields:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'image' field must be a sub message of type 'Image'"
        self._image = value

    @builtins.property
    def compressed_image(self):
        """Message field 'compressed_image'."""
        return self._compressed_image

    @compressed_image.setter
    def compressed_image(self, value):
        if self._check_fields:
            from sensor_msgs.msg import CompressedImage
            assert \
                isinstance(value, CompressedImage), \
                "The 'compressed_image' field must be a sub message of type 'CompressedImage'"
        self._compressed_image = value

    @builtins.property
    def capture_to_publish_ms(self):
        """Message field 'capture_to_publish_ms'."""
        return self._capture_to_publish_ms

    @capture_to_publish_ms.setter
    def capture_to_publish_ms(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'capture_to_publish_ms' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'capture_to_publish_ms' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._capture_to_publish_ms = value
