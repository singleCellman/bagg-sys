from .k4a import _k4a, Device, default_configuration
from .k4arecord import _k4arecord
from .k4arecord.playback import Playback
from .utils import get_k4a_module_path, get_k4arecord_module_path


def initialize_libraries(module_k4a_path=None):
    # Search the module path for k4a if not available
    if module_k4a_path is None:
        module_k4a_path = get_k4a_module_path()

    module_k4arecord_path = get_k4arecord_module_path(module_k4a_path)

    # Initialize k4a related wrappers
    init_k4a(module_k4a_path)

    # Initialize k4arecord related wrappers
    init_k4arecord(module_k4arecord_path)


def init_k4a(module_k4a_path):
    _k4a.setup_library(module_k4a_path)


def init_k4arecord(module_k4arecord_path):
    _k4arecord.setup_library(module_k4arecord_path)


def start_device(device_index=0, config=default_configuration, record=False, record_filepath="output.mkv"):
    # Create device object
    device = Device(device_index)

    # Start device
    res = device.start(config, record, record_filepath)

    return res, device


def start_playback(filepath):
    return Playback(filepath)


