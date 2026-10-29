"""Install the real CLI and exercise Bash's registered completion handlers."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CliCompletionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.home = Path(self.tmp.name)
        self.env = dict(os.environ, HOME=str(self.home),
                        XDG_DATA_HOME=str(self.home / 'data'),
                        PATH=str(self.home / '.local/bin') + ':' + os.environ['PATH'])
        self.install()

    def install(self):
        result = subprocess.run(['bash', str(ROOT / 'scripts/install-cli.sh')],
                                env=self.env, capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)

    def complete(self, *words):
        result = subprocess.run(['bash', '--noprofile', '--norc', '-c', r'''
source /usr/share/bash-completion/bash_completion
complete -W 'topic service' ros2
_completion_loader "$1"
spec=$(complete -p -- "$1") || exit
[[ $spec =~ -F\ ([^\ ]+) ]] || exit 1
handler=${BASH_REMATCH[1]}
COMP_WORDS=("$@")
COMP_CWORD=$(($# - 1))
printf -v COMP_LINE '%s ' "$@"
COMP_LINE=${COMP_LINE% }
COMP_LINE=${COMP_LINE// = /=}
COMP_POINT=${#COMP_LINE}
"$handler" "$1" "${COMP_WORDS[COMP_CWORD]}" "${COMP_WORDS[COMP_CWORD-1]}"
printf '%s\n' "${COMPREPLY[@]}"
''', 'completion-test', *words], cwd=self.home, env=self.env,
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        return set(result.stdout.splitlines()) - {''}

    def test_install_is_repeatable_and_all_commands_load_automatically(self):
        self.install()
        for command in ('srm-nav', 'srm-map', 'srm-pulse', 'srm-rviz'):
            with self.subTest(command=command):
                if (ROOT / 'scripts' / command).is_file():
                    self.assertIn('--help', self.complete(command, '--h'))
                else:
                    self.assertFalse((self.home / '.local/bin' / command).is_symlink())

    def test_navigation_subcommands(self):
        expected = set('status topics hz tf sensors standby field localize drive '
                       'check-standby start prepare cancel reset start-here return-home '
                       'mission-status result standby-down bag-info bag-check map-check '
                       'preview zero map pulse ros help'.split())
        self.assertTrue(expected <= self.complete('srm-nav', ''))
        self.assertEqual(self.complete('srm-nav', 'sta'),
                         {'status', 'standby', 'start', 'start-here', 'standby-down'})

    def test_flags_aliases_and_choice_values(self):
        cases = [
            (('srm-map', '--'), {'--bag', '--duration', '--vehicle-config', '--arena-config',
                                '--output', '--profile', '--response-model', '--help'}),
            (('srm-nav', 'map', '--'), {'--bag', '--profile'}),
            (('srm-nav', 'field', '--'), {'--parking', '--parking-here', '--allow-bypass',
                                       '--no-bypass', '--profile', '--response-model', '--output'}),
            (('srm-nav', 'drive', '--'), {'--allow-bypass', '--no-bypass'}),
            (('srm-nav', 'preview', '--'), {'--all', '--output', '--help'}),
            (('srm-pulse', '--'), {'--device', '--axis', '--value', '--seconds',
                                  '--legacy20', '--wheels-lifted', '--stop-available', '--output'}),
            (('srm-nav', 'pulse', '--a'), {'--axis'}),
            (('srm-pulse', '--axis', ''), {'x', 'y'}),
            (('srm-pulse', ''), {'x', 'y'}),
            (('srm-pulse', '--wheels-lifted', ''), {'x', 'y'}),
            (('srm-pulse', '--axis', 'x', ''), {'x', 'y'}),
            (('srm-map', '--profile', ''), {'safe', 'candidate'}),
            (('srm-nav', 'field', '--profile', ''), {'safe', 'candidate', 'gold-reference'}),
        ]
        for words, expected in cases:
            with self.subTest(words=words):
                self.assertTrue(expected <= self.complete(*words))
        self.assertEqual(self.complete('srm-map', '--profile', 'c'), {'candidate'})
        self.assertEqual(self.complete('srm-map', '--profile', '=', 'c'), {'candidate'})
        for direction in ('x', 'y'):
            self.assertNotIn('x', self.complete('srm-pulse', direction, ''))
            self.assertNotIn('y', self.complete('srm-pulse', direction, ''))
        for words in [('srm-pulse', '--value', '-'),
                      ('srm-nav', 'field', '--parking', '-'),
                      ('srm-nav', 'field', '--parking', '1', '-')]:
            self.assertEqual(self.complete(*words), set())

    def test_ros_wrapper_uses_the_registered_ros2_completer(self):
        self.assertEqual(self.complete('srm-nav', 'ros', 't'), {'topic'})

    def test_paths_preserve_spaces_and_expand_home(self):
        (self.home / 'map space').mkdir()
        (self.home / 'model file.json').touch()
        self.assertIn('map space', self.complete('srm-nav', 'drive', 'map'))
        self.assertIn('model file.json', self.complete('srm-map', '--response-model', 'model'))
        self.assertTrue(any(p.rstrip('/').endswith('/map space') for p in
                            self.complete('srm-nav', 'standby', '~/map')))

    def test_install_refuses_to_replace_another_completion(self):
        target = self.home / 'data/bash-completion/completions/srm-nav'
        target.unlink()
        target.write_text('other completion\n')
        result = subprocess.run(['bash', str(ROOT / 'scripts/install-cli.sh')],
                                env=self.env, capture_output=True, text=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(target.read_text(), 'other completion\n')


if __name__ == '__main__':
    unittest.main()
