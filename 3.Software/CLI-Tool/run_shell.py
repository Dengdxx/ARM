#!/usr/bin/env python3
"""
Robot-Embedded-Framework 命令行工具.

该脚本提供了一个命令行界面 (CLI)，用于与 REF 板（基于 ODrive 或类似硬件）进行交互。
它支持多种命令，例如：
- `shell`: 启动交互式 Python shell。
- `liveplotter`: 实时绘制 REF 参数。
- `drv-status`: 显示驱动器状态寄存器。
- `rate-test`: 测试通信速率。
- `udev-setup`: 设置 Linux udev 规则。
- `generate-code`: 根据设备配置生成代码。
- `backup-config`: 将设备配置备份到文件。
- `restore-config`: 从文件恢复设备配置。

该脚本处理通过 USB 或串行端口进行的设备发现，并管理连接的生命周期。

用法:
    run_shell.py [options] [command] [command_options]

示例:
    ./run_shell.py shell --no-ipython
    ./run_shell.py liveplotter
"""
from __future__ import print_function

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))),
    "Firmware", "fibre", "python"))
from fibre import Logger, Event
import ref_tool
from ref_tool.configuration import *

old_print = print


def print(*args, **kwargs):
    """
    自定义打印函数，用于刷新输出缓冲区。

    这确保了打印语句立即可见，这对于 CLI 工具和日志记录非常重要。

    Args:
        *args: 要打印的可变长度参数列表。
        **kwargs: 任意关键字参数。 'flush' 被弹出并忽略（默认为 False），
                  'file' 默认为 sys.stdout。
    """
    kwargs.pop('flush', False)
    old_print(*args, **kwargs)
    file = kwargs.get('file', sys.stdout)
    file.flush() if file is not None else sys.stdout.flush()


script_path = os.path.dirname(os.path.realpath(__file__))

## 解析参数 ##
parser = argparse.ArgumentParser(description='Robot-Embedded-Framework 命令行工具\n',
                                 formatter_class=argparse.RawTextHelpFormatter)
subparsers = parser.add_subparsers(help='子命令帮助', dest='command')
shell_parser = subparsers.add_parser('shell',
                                     help='进入交互式 Python shell，以便与 ODrive 进行交互')
shell_parser.add_argument("--no-ipython", action="store_true",
                          help="使用常规 Python shell "
                               "而不是 IPython shell，"
                               "即使已安装 IPython。")
subparsers.add_parser('liveplotter', help="用于实时绘制 REF 参数（即位置）")

# 通用参数
parser.add_argument("-p", "--path", metavar="PATH", action="store",
                    help="发现 REF 板的路径。\n"
                         "默认情况下，脚本将连接到 USB 上的任何 REF 板。\n\n"
                         "选择特定的 USB 设备：\n"
                         "  --path usb:BUS:DEVICE\n"
                         "其中 BUS 和 DEVICE 是 `lsusb` 中显示的总线和设备号。\n\n"
                         "选择特定的串行端口：\n"
                         "  --path serial:PATH\n"
                         "其中 PATH 是串行端口的路径。例如 \"/dev/ttyUSB0\"。\n"
                         "您可以使用 `ls /dev/tty*` 查找正确的端口。\n\n"
                         "您可以通过用逗号分隔（无空格！）来组合 USB 和串行规格\n"
                         "示例：\n"
                         "  --path usb,serial:/dev/ttyUSB0\n"
                         "表示 \"发现任何 USB 设备或 /dev/ttyUSB0 上的串行设备\"")
parser.add_argument("-s", "--serial-number", action="store",
                    help="设备的 12 位序列号。"
                         "这是 lsusb 中显示的由 12 个大写十六进制数字组成的字符串。\n"
                         "    示例：385F324D3037\n"
                         "您可以通过运行以下命令列出连接到 USB 的所有设备\n"
                         "(lsusb -d 1209:0d32 -v; lsusb -d 0483:df11 -v) | grep iSerial\n"
                         "如果省略，则接受任何设备。")
parser.add_argument("-v", "--verbose", action="store_true",
                    help="打印调试信息")
parser.add_argument("--version", action="store_true",
                    help="打印版本信息并退出")

parser.set_defaults(path="usb")
args = parser.parse_args()

# 默认命令
if args.command is None:
    args.command = 'shell'
    args.no_ipython = False
logger = Logger(verbose=args.verbose)

app_shutdown_token = Event()

try:
    if args.command == 'shell':
        # if ".dev" in ref_tool.__version__:
        #     print("")
        #     logger.warn("Developer Preview")
        #     print("")
        import ref_tool.shell

        ref_tool.shell.launch_shell(args, logger, app_shutdown_token)

    elif args.command == 'liveplotter':
        from ref_tool.utils import start_liveplotter

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)

        # If you want to plot different values, change them here.
        # You can plot any number of values concurrently.
        cancellation_token = start_liveplotter(lambda: [
            ref_unit.axis0.encoder.pos_estimate,
            ref_unit.axis1.encoder.pos_estimate,
        ])

        print("Showing plot. Press Ctrl+C to exit.")
        while not cancellation_token.is_set():
            time.sleep(1)

    elif args.command == 'drv-status':
        from ref_tool.utils import print_drv_regs

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        print_drv_regs("Motor 0", ref_unit.axis0.motor)
        print_drv_regs("Motor 1", ref_unit.axis1.motor)

    elif args.command == 'rate-test':
        from ref_tool.utils import rate_test

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        rate_test(ref_unit)

    elif args.command == 'udev-setup':
        from ref_tool.version import setup_udev_rules

        setup_udev_rules(logger)

    elif args.command == 'generate-code':
        from ref_tool.code_generator import generate_code

        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     channel_termination_token=app_shutdown_token)
        generate_code(ref_unit, args.template, args.output)

    elif args.command == 'backup-config':
        from ref_tool.configuration import backup_config

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        backup_config(ref_unit, args.file, logger)

    elif args.command == 'restore-config':
        from ref_tool.configuration import restore_config

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        restore_config(ref_unit, args.file, logger)

    else:
        raise Exception("unknown command: " + args.command)

except OperationAbortedException:
    logger.info("Operation aborted.")
finally:
    app_shutdown_token.set()
