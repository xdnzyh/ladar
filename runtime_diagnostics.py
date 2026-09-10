"""Persistent evidence for hardware UI hangs; never sends device commands."""
from datetime import datetime
import faulthandler
import logging
from pathlib import Path


class RuntimeDiagnostics:
    def __init__(self, root, directory: Path, timeout_s: float = 8.0):
        directory.mkdir(parents=True, exist_ok=True)
        self.root = root
        self.timeout_s = timeout_s
        self.callback = None
        self.previous_exception_handler = root.report_callback_exception
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.stack_file = (directory / f'navigation_{stamp}_stacks.log').open('w', encoding='utf-8')
        self.handler = logging.FileHandler(directory / f'navigation_{stamp}.log', encoding='utf-8')
        self.handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
        self.logger = logging.getLogger('navigation.runtime')
        self.previous_level = self.logger.level
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(self.handler)
        root.report_callback_exception = self.report_exception
        self.logger.info('诊断启动：界面连续 %.1f 秒未响应时记录所有线程堆栈', timeout_s)
        self.heartbeat()

    def heartbeat(self):
        # The native watchdog still fires if the Tk thread is blocked on a lock.
        faulthandler.dump_traceback_later(self.timeout_s, file=self.stack_file, repeat=False)
        self.callback = self.root.after(max(1, int(self.timeout_s * 125)), self.heartbeat)

    def report_exception(self, exc_type, value, tb):
        self.logger.error('Tk 回调异常', exc_info=(exc_type, value, tb))
        self.previous_exception_handler(exc_type, value, tb)

    def close(self):
        faulthandler.cancel_dump_traceback_later()
        if self.callback is not None:
            try:
                self.root.after_cancel(self.callback)
            except Exception:
                pass  # The window may already have been destroyed.
        self.root.report_callback_exception = self.previous_exception_handler
        self.logger.removeHandler(self.handler)
        self.logger.setLevel(self.previous_level)
        self.handler.close()
        self.stack_file.close()
