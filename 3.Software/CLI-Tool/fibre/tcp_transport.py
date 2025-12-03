
import sys
import socket
import time
import traceback
import fibre.protocol
from fibre.utils import wait_any, TimeoutError

def noprint(x):
  """
  无操作打印函数。

  Args:
      x: 要打印的参数（被忽略）。
  """
  pass

class TCPTransport(fibre.protocol.StreamSource, fibre.protocol.StreamSink):
  """
  通过 TCP 连接实现 StreamSource 和 StreamSink。
  """
  def __init__(self, dest_addr, dest_port, logger):
    """
    初始化 TCP 传输并连接到目的地。

    Args:
        dest_addr (str): 目标主机名或 IP 地址。
        dest_port (int): 目标端口。
        logger (Logger): 记录器实例。
    """
    # TODO: FIXME: use IPv6
    # Problem: getaddrinfo fails if the resolver returns an
    # IPv4 address, but we are using AF_INET6
    #family = socket.AF_INET6 if socket.has_ipv6 else socket.AF_INET
    family = socket.AF_INET
    self.sock = socket.socket(family, socket.SOCK_STREAM)
    # TODO: Determine the right address to use from the list
    self.target = socket.getaddrinfo(dest_addr, dest_port, family)[0][4]
    # TODO: this blocks until a connection is established, or the system cancels it
    self.sock.connect(self.target)

  def process_bytes(self, buffer):
    """
    通过 TCP 连接发送字节。

    Args:
        buffer (bytes): 要发送的数据。
    """
    self.sock.send(buffer)

  def get_bytes(self, n_bytes, deadline):
    """
    返回 n 个字节，除非达到截止日期，在这种情况下返回此时读取的字节。
    如果截止日期为 None，则函数将永远阻塞。
    当前时间之前的截止日期对应于非阻塞模式。

    Args:
        n_bytes (int): 要读取的字节数。
        deadline (float): 时间戳截止日期。

    Returns:
        bytes: 读取的数据。

    Raises:
        TimeoutError: 如果超出截止日期（在 get_bytes_or_fail 内部）。
    """
    # convert deadline to seconds (floating point)
    deadline = None if deadline is None else max(deadline - time.monotonic(), 0)
    self.sock.settimeout(deadline)
    try:
      data = self.sock.recv(n_bytes) # receive n_bytes
      return data
    except socket.timeout:
      # if we got a timeout data will still be none, so we call recv again
      # this time in non blocking state and see if we can get some data
      try:
        return self.sock.recv(n_bytes)
      except socket.timeout:
        raise TimeoutError

  def get_bytes_or_fail(self, n_bytes, deadline):
    """
    尝试在截止日期之前读取确切的 n 个字节。

    Args:
        n_bytes (int): 要读取的字节数。
        deadline (float): 时间戳截止日期。

    Returns:
        bytes: 读取的数据。

    Raises:
        TimeoutError: 如果读取的字节数少于 n_bytes。
    """
    result = self.get_bytes(n_bytes, deadline)
    if len(result) < n_bytes:
      raise TimeoutError("expected {} bytes but got only {}".format(n_bytes, len(result)))
    return result



def discover_channels(path, serial_number, callback, cancellation_token, channel_termination_token, logger):
  """
  尝试基于路径规范连接到 TCP 服务器。
  此函数阻塞直到设置了 cancellation_token。
  由此函数生成的通道运行，直到设置了 channel_termination_token。

  Args:
      path (str): 格式为 "host:port" 的路径规范。
      serial_number (str): (未使用)。
      callback (callable): 用于调用发现通道的函数。
      cancellation_token (Event): 停止发现的信号。
      channel_termination_token (Event): 终止通道的信号。
      logger (Logger): 记录器实例。
  """
  try:
    dest_addr = ':'.join(path.split(":")[:-1])
    dest_port = int(path.split(":")[-1])
  except (ValueError, IndexError):
    raise Exception('"{}" is not a valid TCP destination. The format should be something like "localhost:1234".'
                    .format(path))

  while not cancellation_token.is_set():
    try:
      tcp_transport = fibre.tcp_transport.TCPTransport(dest_addr, dest_port, logger)
      stream2packet_input = fibre.protocol.PacketFromStreamConverter(tcp_transport)
      packet2stream_output = fibre.protocol.StreamBasedPacketSink(tcp_transport)
      channel = fibre.protocol.Channel(
              "TCP device {}:{}".format(dest_addr, dest_port),
              stream2packet_input, packet2stream_output,
              channel_termination_token, logger)
    except:
      #logger.debug("TCP channel init failed. More info: " + traceback.format_exc())
      pass
    else:
      callback(channel)
      wait_any(None, cancellation_token, channel._channel_broken)
    time.sleep(1)
