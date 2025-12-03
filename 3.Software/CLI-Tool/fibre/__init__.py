"""
Fibre: 嵌入式设备的通信库。

该软件包提供了通过各种传输（USB、串行、TCP、UDP）发现和与实现 Fibre 协议的设备进行通信的工具。
"""

from .discovery import find_any, find_all
from .utils import Event, Logger, TimeoutError
from .protocol import ChannelBrokenException, ChannelDamagedException
from .shell import launch_shell
