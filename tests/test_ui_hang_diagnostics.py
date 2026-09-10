import logging
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock

from runtime_diagnostics import RuntimeDiagnostics


class UIHangDiagnosticTests(unittest.TestCase):
    def test_blocked_ui_leaves_all_thread_stack_without_ui_callback(self):
        with tempfile.TemporaryDirectory() as directory:
            script = '''
import sys, time
from pathlib import Path
from unittest.mock import Mock
from runtime_diagnostics import RuntimeDiagnostics
diagnostics = RuntimeDiagnostics(Mock(), Path(sys.argv[1]), timeout_s=0.1)
try:
    time.sleep(0.4)  # No heartbeat: emulate the blocked UI thread.
finally:
    diagnostics.close()
'''
            result = subprocess.run([sys.executable, '-c', script, directory],
                                    cwd=Path(__file__).resolve().parents[1],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            stacks = next(Path(directory).glob('*_stacks.log')).read_text(encoding='utf-8')
            self.assertIn('Timeout', stacks)
            self.assertIn('<string>', stacks)

    def test_status_and_callback_exception_are_persisted_and_handler_restored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Mock()
            previous = root.report_callback_exception
            diagnostics = RuntimeDiagnostics(root, Path(directory))
            try:
                logging.getLogger('navigation.runtime').info('底盘故障 BAD_CMD')
                try:
                    raise ValueError('test callback failure')
                except ValueError:
                    root.report_callback_exception(*sys.exc_info())
            finally:
                diagnostics.close()
            self.assertIs(root.report_callback_exception, previous)
            previous.assert_called_once()
            log = next(path for path in Path(directory).glob('*.log') if '_stacks' not in path.name)
            content = log.read_text(encoding='utf-8')
            self.assertIn('底盘故障 BAD_CMD', content)
            self.assertIn('ValueError: test callback failure', content)


if __name__ == '__main__':
    unittest.main()
