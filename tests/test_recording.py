"""Real filesystem ordering, complete shutdown drain and propagated writer errors."""
from pathlib import Path
import tempfile
import unittest
from intramural_mission.recording import StatusLog


class StatusLogTests(unittest.TestCase):
    def test_close_preserves_all_lines_in_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'status.jsonl';log=StatusLog(path)
            lines=[str(i)+' '+('x'*5000)+'\n' for i in range(100)]
            for line in lines:log.write(line)
            snapshot=Path(directory)/'route.json';log.snapshot(snapshot,'{"route": 1}\n')
            log.close();log.close()
            self.assertEqual(path.read_text(),''.join(lines))
            self.assertEqual(snapshot.read_text(),'{"route": 1}\n')
            with self.assertRaises(ValueError):log.write('late\n')

    @unittest.skipUnless(Path('/dev/full').exists(),'Requires the kernel ENOSPC test device')
    def test_disk_failure_is_reported_on_shutdown(self):
        log=StatusLog(Path('/dev/full'));log.write('status\n')
        with self.assertRaises(OSError):log.close()
