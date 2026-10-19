"""Exercise graphics startup at the external GLX executable boundary."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class GraphicsEntryTest(unittest.TestCase):
    def test_preflight_runs_application_only_with_usable_graphics(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=root / 'cache') as temporary:
            folder = Path(temporary)
            probe = folder / 'glxinfo'
            marker = folder / 'started'
            env = {**os.environ, 'PATH': str(folder) + os.pathsep + os.environ['PATH']}
            for output, probe_exit, software, starts in (
                ('Accelerated: yes', 0, '0', True),
                ('Accelerated: no', 0, '0', False),
                ('Accelerated: no', 0, '1', True),
                ('', 42, '0', False),
            ):
                with self.subTest(output=output, probe_exit=probe_exit, software=software):
                    marker.unlink(missing_ok=True)
                    probe.write_text(f'#!/bin/sh\necho "{output}"\nexit {probe_exit}\n')
                    probe.chmod(0o755)
                    result = subprocess.run(
                        ['bash', str(root / 'scripts/internal/graphics-entrypoint.sh'),
                         'touch', str(marker)],
                        env={**env, 'LIBGL_ALWAYS_SOFTWARE': software},
                        capture_output=True, text=True, timeout=5)
                    self.assertEqual(result.returncode, 0 if starts else 1)
                    self.assertEqual(marker.exists(), starts)


if __name__ == '__main__':
    unittest.main()
