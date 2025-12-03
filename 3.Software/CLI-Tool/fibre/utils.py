
import sys
import time
import threading
import platform
import subprocess
import os

try:
    if platform.system() == 'Windows':
        import win32console
        # TODO: we should win32console anyway so we could just omit colorama
        import colorama
        colorama.init()
except ImportError:
    print("Could not init terminal features.")
    sys.stdout.flush()
    pass

if sys.version_info < (3, 3):
    class TimeoutError(Exception):
        pass
else:
    TimeoutError = TimeoutError

def get_serial_number_str(device):
    """
    检索设备的序列号作为十六进制字符串。

    Args:
        device (RemoteObject): 要检查的设备。

    Returns:
        str: 12 位十六进制序列号或占位符。
    """
    if hasattr(device, 'serial_number'):
        return format(device.serial_number, 'x').upper()
    else:
        return "[unknown serial number]"

## Threading utils ##
class Event():
    """
    threading.Event() 的替代方案，通过提供原始版本未能提供的 subscribe() 函数进行了增强。
    """
    def __init__(self, trigger=None):
        """
        初始化 Event。

        Args:
            trigger (Event): 如果提供，则当设置触发事件时，将触发此新事件。
        """
        self._evt = threading.Event()
        self._subscribers = []
        self._mutex = threading.Lock()
        if not trigger is None:
            trigger.subscribe(lambda: self.set())

    def is_set(self):
        """
        检查事件是否已设置。

        Returns:
            bool: 如果已设置则为 True，否则为 False。
        """
        return self._evt.is_set()

    def set(self):
        """
        设置事件并调用所有订阅者（如果尚未设置事件）。
        """
        self._mutex.acquire()
        try:
            if not self._evt.is_set():
                self._evt.set()
                for s in self._subscribers:
                    s()
        finally:
            self._mutex.release()

    def subscribe(self, handler):
        """
        一旦设置了指定事件，就立即调用指定的处理程序一次。
        如果事件已设置，则立即调用处理程序。

        Args:
            handler (callable): 设置事件时要调用的函数。

        Returns:
            callable: 处理程序函数（对于取消订阅很有用）。
        """
        if handler is None:
            raise TypeError
        self._mutex.acquire()
        try:
            self._subscribers.append(handler)
            if self._evt.is_set():
                handler()
        finally:
            self._mutex.release()
        return handler

    def unsubscribe(self, handler):
        """
        删除订阅者。

        Args:
            handler (callable): 要删除的处理程序。
        """
        self._mutex.acquire()
        try:
            self._subscribers.pop(self._subscribers.index(handler))
        finally:
            self._mutex.release()

    def wait(self, timeout=None):
        """
        等待事件被设置。

        Args:
            timeout (float): 等待的最长时间。

        Raises:
            TimeoutError: 如果发生超时。
        """
        if not self._evt.wait(timeout=timeout):
            raise TimeoutError()

    def trigger_after(self, timeout):
        """
        在指定的超时后触发事件。
        此函数立即返回。

        Args:
            timeout (float): 触发前的时间（以秒为单位）。
        """
        def delayed_trigger():
            if not self.wait(timeout=timeout):
                self.set()
        threading.Thread(target=delayed_trigger)
        t.daemon = True
        t.start()


def wait_any(timeout=None, *events):
    """
    阻塞直到触发任何指定的事件。

    Args:
        timeout (float): 超时（以秒为单位）。
        *events: 可变数量的 Event 对象。

    Returns:
        int: 触发事件的索引。

    Raises:
        TimeoutError: 如果在设置任何事件之前超时。
    """
    or_event = threading.Event()
    subscriptions = []
    for event in events:
        subscriptions.append((event, event.subscribe(lambda: or_event.set())))
    or_event.wait(timeout=timeout)
    for event, sub in subscriptions:
        event.unsubscribe(sub)
    for i in range(len(events)):
        if events[i].is_set():
            return i
    raise TimeoutError()


## Log utils ##

class Logger():
    """
    将消息记录到 stdout，具有可选颜色和处理在交互式提示上方打印的功能。
    """

    COLOR_DEFAULT = 0
    COLOR_GREEN = 1
    COLOR_CYAN = 2
    COLOR_YELLOW = 3
    COLOR_RED = 4

    _VT100Colors = {
        COLOR_GREEN: '\x1b[92;1m',
        COLOR_CYAN: '\x1b[96;1m',
        COLOR_YELLOW: '\x1b[93;1m',
        COLOR_RED: '\x1b[91;1m',
        COLOR_DEFAULT: '\x1b[0m'
    }

    _Win32Colors = {
        COLOR_GREEN: 0x0A,
        COLOR_CYAN: 0x0B,
        COLOR_YELLOW: 0x0E,
        COLOR_RED: 0x0C,
        COLOR_DEFAULT: 0x07
    }

    def __init__(self, verbose=True):
        """
        初始化记录器。

        Args:
            verbose (bool): 如果为 True，则打印调试消息。
        """
        self._prefix = ''
        self._skip_bottom_line = False # If true, messages are printed one line above the cursor
        self._verbose = verbose
        self._print_lock = threading.Lock()
        if platform.system() == 'Windows':
            self._stdout_buf = win32console.GetStdHandle(win32console.STD_OUTPUT_HANDLE)

    def indent(self, prefix='  '):
        """
        创建一个缩进其输出的新 Logger 实例。

        Args:
            prefix (str): 预先添加到消息的字符串。

        Returns:
            Logger: 一个新的缩进记录器。
        """
        indented_logger = Logger()
        indented_logger._prefix = self._prefix + prefix
        return indented_logger

    def print_on_second_last_line(self, text, color):
        """
        在倒数第二行打印文本。
        这可用于在命令提示符上方打印消息。
        如果命令提示符跨越多行，则会出现故障。
        如果打印的文本跨越多行，也会出现故障（尽管这可以修复）。

        Args:
            text (str): 要打印的文本。
            color (int): 颜色代码。
        """

        if platform.system() == 'Windows':
            # Windows <10 doesn't understand VT100 escape codes and the colorama
            # also doesn't support the specific escape codes we need so we use the
            # native Win32 API.
            info = self._stdout_buf.GetConsoleScreenBufferInfo()
            cursor_pos = info['CursorPosition']
            scroll_rect=win32console.PySMALL_RECTType(
                Left=0, Top=1,
                Right=info['Window'].Right,
                Bottom=cursor_pos.Y-1)
            scroll_dest = win32console.PyCOORDType(scroll_rect.Left, scroll_rect.Top-1)
            self._stdout_buf.ScrollConsoleScreenBuffer(
                scroll_rect, scroll_rect, scroll_dest, # clipping rect is same as scroll rect
                u' ', Logger._Win32Colors[color]) # fill with empty cells with the desired color attributes
            line_start = win32console.PyCOORDType(0, cursor_pos.Y-1)
            self._stdout_buf.WriteConsoleOutputCharacter(text, line_start)

        else:
            # Assume we're in a terminal that interprets VT100 escape codes.
            # TODO: test on macOS

            # Escape character sequence:
            #   ESC 7: store cursor position
            #   ESC 1A: move cursor up by one
            #   ESC 1S: scroll entire viewport by one
            #   ESC 1L: insert 1 line at cursor position
            #   (print text)
            #   ESC 8: restore old cursor position

            self._print_lock.acquire()
            sys.stdout.write('\x1b7\x1b[1A\x1b[1S\x1b[1L')
            sys.stdout.write(Logger._VT100Colors[color] + text + Logger._VT100Colors[Logger.COLOR_DEFAULT])
            sys.stdout.write('\x1b8')
            sys.stdout.flush()
            self._print_lock.release()

    def print_colored(self, text, color):
        """
        将彩色文本打印到 stdout。

        Args:
            text (str): 要打印的文本。
            color (int): 颜色代码。
        """
        if self._skip_bottom_line:
            self.print_on_second_last_line(text, color)
        else:
            # On Windows, colorama does the job of interpreting the VT100 escape sequences
            self._print_lock.acquire()
            sys.stdout.write(Logger._VT100Colors[color] + text + Logger._VT100Colors[Logger.COLOR_DEFAULT] + '\n')
            sys.stdout.flush()
            self._print_lock.release()

    def debug(self, text):
        """记录调试消息（如果详细）。"""
        if self._verbose:
            self.print_colored(self._prefix + text, Logger.COLOR_DEFAULT)
    def success(self, text):
        """以绿色记录成功消息。"""
        self.print_colored(self._prefix + text, Logger.COLOR_GREEN)
    def info(self, text):
        """记录信息消息。"""
        self.print_colored(self._prefix + text, Logger.COLOR_DEFAULT)
    def notify(self, text):
        """以青色记录通知。"""
        self.print_colored(self._prefix + text, Logger.COLOR_CYAN)
    def warn(self, text):
        """以黄色记录警告。"""
        self.print_colored(self._prefix + text, Logger.COLOR_YELLOW)
    def error(self, text):
        """以红色记录错误。"""
        # TODO: write to stderr
        self.print_colored(self._prefix + text, Logger.COLOR_RED)
