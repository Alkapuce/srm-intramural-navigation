import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class LidarViewCliTest(unittest.TestCase):
    def test_window_exit_cleans_up_stream_and_missing_topic_fails(self):
        source = Path(__file__).resolve().parents[1] / 'scripts/srm-rviz'
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'scripts').mkdir()
            (root / 'bin').mkdir()
            shutil.copy(source, root / 'scripts/srm-rviz')
            ssh = root / 'bin/ssh'
            ssh.write_text('''#!/bin/bash
if [[ "$*" == *"topic info"* ]]; then
  echo "Publisher count: ${FAKE_PUBLISHERS:-1}"
  exit 0
fi
echo $$ > "$TEST_ROOT/ssh.pid"
exec sleep 60
''')
            runner = root / 'scripts/run.sh'
            runner.write_text('''#!/bin/bash
if [[ "$*" == *"--receive"* ]]; then
  echo $$ > "$TEST_ROOT/receiver.pid"
  exec sleep 60
fi
for ((i=0; i<100; i++)); do
  [[ -f "$TEST_ROOT/ssh.pid" && -f "$TEST_ROOT/receiver.pid" ]] && exit 0
  sleep .02
done
exit 1
''')
            ssh.chmod(0o755)
            runner.chmod(0o755)
            env = dict(os.environ, PATH=str(root / 'bin') + ':' + os.environ['PATH'], TEST_ROOT=tmp)
            result = subprocess.run([str(root / 'scripts/srm-rviz')], env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            for name in ('ssh.pid', 'receiver.pid'):
                pid = int((root / name).read_text())
                with self.assertRaises(ProcessLookupError):
                    os.kill(pid, 0)
            result = subprocess.run([str(root / 'scripts/srm-rviz')], env=dict(env, FAKE_PUBLISHERS='0'), capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 1)
            self.assertIn('点云未就绪', result.stderr)


if __name__ == '__main__':
    unittest.main()
