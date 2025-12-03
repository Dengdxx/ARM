"""
提供发现 Fibre 节点的功能。
"""

import sys
import json
import time
import threading
import traceback
import fibre.protocol
import fibre.utils
import fibre.remote_object
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException, TimeoutError

# Load all installed transport layers

channel_types = {}

try:
    import fibre.usbbulk_transport
    channel_types['usb'] = fibre.usbbulk_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.serial_transport
    channel_types['serial'] = fibre.serial_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.tcp_transport
    channel_types['tcp'] = fibre.tcp_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.udp_transport
    channel_types['udp'] = fibre.udp_transport.discover_channels
except ImportError:
    pass

def noprint(text):
    """
    一个无操作的打印函数。

    Args:
        text: 将被打印的文本。
    """
    pass

def find_all(path, serial_number,
         did_discover_object_callback,
         search_cancellation_token,
         channel_termination_token,
         logger):
    """
    开始扫描与指定路径规范匹配的 Fibre 节点，并为找到的每个 Fibre 节点调用回调函数。
    此函数是非阻塞的。

    Args:
        path (str): 搜索路径规范（例如 'usb', 'serial:/dev/ttyUSB0'）。
        serial_number (str): 要筛选的设备序列号。
        did_discover_object_callback (callable): 发现设备时调用的函数。
                                                它接收发现的对象作为参数。
        search_cancellation_token (Event): 用于发出取消搜索信号的事件。
        channel_termination_token (Event): 用于发出终止通道信号的事件。
        logger (Logger): 用于调试输出的记录器实例。

    Returns:
        None
    """

    def did_discover_channel(channel):
        """
        从给定的通道初始化一个对象，然后调用 did_discover_object_callback
        并传入创建的对象。
        这将查询该通道上的端点 0 以获取有关接口的信息，然后用于初始化相应的对象。

        Args:
            channel (Channel): 发现的通信通道。
        """
        try:
            logger.debug("Connecting to device on " + channel._name)
            try:
                json_bytes = channel.remote_endpoint_read_buffer(0)
            except (TimeoutError, ChannelBrokenException):
                logger.debug("no response - probably incompatible")
                return
            json_crc16 = fibre.protocol.calc_crc16(fibre.protocol.PROTOCOL_VERSION, json_bytes)
            channel._interface_definition_crc = json_crc16
            try:
                json_string = json_bytes.decode("ascii")
            except UnicodeDecodeError:
                logger.debug("device responded on endpoint 0 with something that is not ASCII")
                return
            logger.debug("JSON: " + json_string.replace('{"name"', '\n{"name"'))
            logger.debug("JSON checksum: 0x{:02X} 0x{:02X}".format(json_crc16 & 0xff, (json_crc16 >> 8) & 0xff))
            try:
                json_data = json.loads(json_string)
            except json.decoder.JSONDecodeError as error:
                logger.debug("device responded on endpoint 0 with something that is not JSON: " + str(error))
                return
            json_data = {"name": "fibre_node", "members": json_data}
            obj = fibre.remote_object.RemoteObject(json_data, None, channel, logger)

            obj.__dict__['_json_data'] = json_data['members']
            obj.__dict__['_json_crc'] = json_crc16

            device_serial_number = fibre.utils.get_serial_number_str(obj)
            if serial_number != None and device_serial_number != serial_number:
                logger.debug("Ignoring device with serial number {}".format(device_serial_number))
                return
            did_discover_object_callback(obj)
        except Exception:
            logger.debug("Unexpected exception after discovering channel: " + traceback.format_exc())

    # For each connection type, kick off an appropriate discovery loop
    for search_spec in path.split(','):
        prefix = search_spec.split(':')[0]
        the_rest = ':'.join(search_spec.split(':')[1:])
        if prefix in channel_types:
            t = threading.Thread(target=channel_types[prefix],
                             args=(the_rest, serial_number, did_discover_channel, search_cancellation_token, channel_termination_token, logger))
            t.daemon = True
            t.start()
        else:
            raise Exception("Invalid path spec \"{}\"".format(search_spec))


def find_any(path="usb", serial_number=None,
        search_cancellation_token=None, channel_termination_token=None,
        timeout=None, logger=Logger(verbose=False)):
    """
    阻塞直到连接第一个匹配的 Fibre 节点，然后返回该节点。

    Args:
        path (str): 搜索路径规范（默认为 "usb"）。
        serial_number (str): 要筛选的设备序列号（可选）。
        search_cancellation_token (Event): 用于发出取消搜索信号的事件（可选）。
        channel_termination_token (Event): 用于发出终止通道信号的事件（可选）。
        timeout (float): 等待设备的最长时间（以秒为单位）（可选）。
        logger (Logger): 要使用的记录器实例（默认为非详细记录器）。

    Returns:
        RemoteObject: 发现的远程对象（Fibre 节点），如果未找到/超时，则为 None。
    """
    result = [ None ]
    done_signal = Event(search_cancellation_token)
    def did_discover_object(obj):
        result[0] = obj
        done_signal.set()
    find_all(path, serial_number, did_discover_object, done_signal, channel_termination_token, logger)
    try:
        done_signal.wait(timeout=timeout)
    finally:
        done_signal.set() # terminate find_all
    return result[0]
