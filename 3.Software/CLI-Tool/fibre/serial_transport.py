"""
提供为串行端口实现 StreamSource/StreamSink 和 PacketSource/PacketSink 接口的类。
"""

import os
import re
import time
import traceback
import serial
import serial.tools.list_ports
import fibre
from fibre.utils import TimeoutError

# TODO: make this customizable
DEFAULT_BAUDRATE = 115200

class SerialStreamTransport(fibre.protocol.StreamSource, fibre.protocol.StreamSink):
    """
    pyserial 的包装器，提供 StreamSource 和 StreamSink 接口。
    """
    def __init__(self, port, baud):
        """
        初始化串行传输。

        Args:
            port (str): 串行端口路径（例如 '/dev/ttyUSB0' 或 'COM3'）。
            baud (int): 波特率。
        """
        self._dev = serial.Serial(port, baud, timeout=1)

    def process_bytes(self, bytes):
        """
        将字节发送到串行端口。

        Args:
            bytes (bytes): 要发送的数据。
        """
        self._dev.write(bytes)

    def get_bytes(self, n_bytes, deadline):
        """
        返回 n 个字节，除非达到截止日期，在这种情况下返回此时读取的字节。
        如果截止日期为 None，则函数将永远阻塞。
        当前时间之前的截止日期对应于非阻塞模式。

        Args:
            n_bytes (int): 要读取的字节数。
            deadline (float): 截止时间戳。

        Returns:
            bytes: 从串行端口读取的字节。
        """
        if deadline is None:
            self._dev.timeout = None
        else:
            self._dev.timeout = max(deadline - time.monotonic(), 0)
        return self._dev.read(n_bytes)

    def get_bytes_or_fail(self, n_bytes, deadline):
        """
        尝试在截止日期之前确切读取 n 个字节。

        Args:
            n_bytes (int): 要读取的字节数。
            deadline (float): 截止时间戳。

        Returns:
            bytes: 读取的字节。

        Raises:
            TimeoutError: 如果在截止日期之前读取的字节数少于 n_bytes。
        """
        result = self.get_bytes(n_bytes, deadline)
        if len(result) < n_bytes:
            raise TimeoutError("expected {} bytes but got only {}", n_bytes, len(result))
        return result

    def close(self):
        """
        关闭串行端口。
        """
        self._dev.close()


def find_dev_serial_ports():
    """
    列出 /dev 中的串行端口。

    Returns:
        list[str]: 潜在串行端口的路径列表。
    """
    try:
        return ['/dev/' + x for x in os.listdir('/dev')]
    except FileNotFoundError:
        return []

def find_pyserial_ports():
    """
    列出 pyserial 找到的串行端口。

    Returns:
        list[str]: 设备名称列表（例如 'COM3', '/dev/ttyUSB0'）。
    """
    return [x.device for x in serial.tools.list_ports.comports()]


def discover_channels(path, serial_number, callback, cancellation_token, channel_termination_token, logger):
    """
    扫描与路径规范匹配的串行端口。
    此函数阻塞直到设置了 cancellation_token。
    由此函数生成的通道运行，直到设置了 channel_termination_token。

    Args:
        path (str): 路径正则表达式，或 None 以匹配标准串行端口。
        serial_number (str): (当前在串行发现中未使用)。
        callback (callable): 使用发现的 Channel 调用的函数。
        cancellation_token (Event): 停止发现的信号。
        channel_termination_token (Event): 终止通道的信号。
        logger (Logger): 记录器对象。
    """
    if path == None:
        # This regex should match all desired port names on macOS,
        # Linux and Windows but might match some incorrect port names.
        regex = r'^(/dev/tty\.usbmodem.*|/dev/ttyACM.*|COM[0-9]+)$'
    else:
        regex = "^" + path + "$"

    known_devices = []
    def device_matcher(port_name):
        if port_name in known_devices:
            return False
        return bool(re.match(regex, port_name))

    def did_disconnect(port_name, device):
        device.close()
        # TODO: yes there is a race condition here in case you wonder.
        known_devices.pop(known_devices.index(port_name))

    while not cancellation_token.is_set():
        all_ports = find_pyserial_ports() + find_dev_serial_ports()
        new_ports = filter(device_matcher, all_ports)
        for port_name in new_ports:
            try:
                serial_device = SerialStreamTransport(port_name, DEFAULT_BAUDRATE)
                input_stream = fibre.protocol.PacketFromStreamConverter(serial_device)
                output_stream = fibre.protocol.StreamBasedPacketSink(serial_device)
                channel = fibre.protocol.Channel(
                        "serial port {}@{}".format(port_name, DEFAULT_BAUDRATE),
                        input_stream, output_stream, channel_termination_token, logger)
                channel.serial_device = serial_device
            except serial.serialutil.SerialException:
                logger.debug("Serial device init failed. Ignoring this port. More info: " + traceback.format_exc())
                known_devices.append(port_name)
            else:
                known_devices.append(port_name)
                channel._channel_broken.subscribe(lambda: did_disconnect(port_name, serial_device))
                callback(channel)
        time.sleep(1)
