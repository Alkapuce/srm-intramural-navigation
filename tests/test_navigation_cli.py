"""The real launch entry must reject a second vehicle stack before opening devices."""

import fcntl
import os
from pathlib import Path
import subprocess
import unittest


class NavigationCliTests(unittest.TestCase):
    def test_existing_navigation_lock_rejects_another_launch(self):
        root = Path(__file__).resolve().parents[1]
        lock = root / 'cache/runtime/vehicle-navigation.lock'
        lock.parent.mkdir(parents=True, exist_ok=True)
        with lock.open('a') as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                pass  # An existing real instance already provides the competing lock.
            result = subprocess.run(
                [str(root / 'scripts/vehicle-navigation.sh')],
                env=dict(os.environ, INTRAMURAL_NAVIGATION_CONFIG='not-opened'),
                capture_output=True, text=True, timeout=5,
            )
            self.assertEqual(result.returncode, 75, result.stderr)


if __name__ == '__main__':
    unittest.main()
