
import sys
import socket
import time
import traceback
import fibre.protocol
from fibre.utils import wait_any

def noprint(x):
  """
  无操作打印函数。

  Args:
      x: 要打印的参数（被忽略）。
  """
  pass

class UDPTransport(fibre.protocol.PacketSource, fibre.protocol.PacketSink):
  """
  通过 UDP 实现 PacketSource 和 PacketSink。
  """
  def __init__(self, dest_addr, dest_port, logger):
    """
    初始化 UDP 传输。

    Args:
        dest_addr (str): 目标地址（主机名或 IP）。
        dest_port (int): 目标端口。
        logger (Logger): 记录器实例。
    """
    # TODO: FIXME: use IPv6
    # Problem: getaddrinfo fails if the resolver returns an
    # IPv4 address, but we are using AF_INET6
    #family = socket.AF_INET6 if socket.has_ipv6 else socket.AF_INET
    family = socket.AF_INET
    self.sock = socket.socket(family, socket.SOCK_DGRAM)
    # TODO: Determine the right address to use from the list
    self.target = socket.getaddrinfo(dest_addr,dest_port, family)[0][4]

  def process_packet(self, buffer):
    """
    通过 UDP 发送数据包。

    Args:
        buffer (bytes): 数据包数据。
    """
    self.sock.sendto(buffer, self.target)

  def get_packet(self, deadline):
    """
    通过 UDP 接收数据包。

    Args:
        deadline (float): 时间戳截止日期。

    Returns:
        bytes: 接收到的数据包数据。

    Raises:
        TimeoutError: 如果在截止日期之前未收到数据包。
    """
    # TODO: implement deadline
    deadline = None if deadline is None else max(deadline - time.monotonic(), 0)
    self.sock.settimeout(deadline)
    try:
      data, _ = self.sock.recvfrom(1024)
      return data
    except socket.timeout:
      # if we got a timeout data will still be none, so we call recv again
      # this time in non blocking state and see if we can get some data
      try:
        return self.sock.recvfrom(1024)
      except socket.timeout:
        raise TimeoutError

def discover_channels(path, serial_number, callback, cancellation_token, channel_termination_token, logger):
  """
  尝试基于路径规范连接到 UDP 服务器。
  此函数阻塞直到设置了 cancellation_token。
  由此函数生成的通道运行，直到设置了 channel_termination_token。

  Args:
      path (str): 路径规范 "host:port"。
      serial_number (str): (未使用)。
      callback (callable): 使用发现的通道调用的函数。
      cancellation_token (Event): 停止发现的信号。
      channel_termination_token (Event): 终止通道的信号。
      logger (Logger): 记录器实例。
  """
  try:
    dest_addr = ':'.join(path.split(":")[:-1])
    dest_port = int(path.split(":")[-1])
  except (ValueError, IndexError):
    raise Exception('"{}" is not a valid UDP destination. The format should be something like "localhost:1234".'
                    .format(path))

  while not cancellation_token.is_set():
    try:
      udp_transport = fibre.udp_transport.UDPTransport(dest_addr, dest_port, logger)
      channel = fibre.protocol.Channel(
              "UDP device {}:{}".format(dest_addr, dest_port),
              udp_transport, udp_transport,
              channel_termination_token, logger)
    except:
      logger.debug("UDP channel init failed. More info: " + traceback.format_exc())
      pass
    else:
      callback(channel)
      wait_any(None, cancellation_token, channel._channel_broken)
    time.sleep(1)
