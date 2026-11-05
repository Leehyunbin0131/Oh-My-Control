"""Verify network state changes with mocked commands; never switch real Wi-Fi."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import robot_hotspot as hotspot


class HotspotTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.messages = io.StringIO()
        self.stack = []
        for name, replacement in [('run', self.run_command), ('value', self.value),
                                  ('install_service', lambda: None)]:
            p = patch.object(hotspot, name, replacement)
            p.start(); self.addCleanup(p.stop)

    def run_command(self, *args, **kwargs):
        self.calls.append(args)
        return SimpleNamespace(returncode=3 if 'is-active' in args else 0, stdout='', stderr='')

    def value(self, field, *args, **kwargs):
        return {'WIFI-PROPERTIES.AP': 'yes', 'connection.uuid': 'ap-uuid',
                'GENERAL.CON-UUID': 'indoor-uuid', '802-11-wireless.ssid': 'indoor',
                '802-11-wireless-security.psk': 'test-secret',
                'IP4.ADDRESS': '192.168.0.37/24', 'connection.autoconnect': 'yes'}.get(field, '')

    def test_enable_sets_password_persistence_and_ssh_independent_switch(self):
        with redirect_stdout(self.messages):
            hotspot.enable()
        modify = next(c for c in self.calls if c[:3] == ('nmcli', 'connection', 'modify'))
        values = dict(zip(modify[5::2], modify[6::2]))
        self.assertEqual(values['802-11-wireless-security.psk'], '12345678')
        self.assertEqual(values['ipv4.addresses'], '10.42.0.1/24')
        self.assertEqual(values['connection.autoconnect'], 'yes')
        self.assertEqual(values['ipv4.method'], 'shared')
        self.assertIn(('systemctl', 'enable', hotspot.SERVICE), self.calls)
        scheduled = next(c for c in self.calls if c[0] == 'systemd-run')
        self.assertIn('--on-active=5s', scheduled)
        self.assertIn('ssh hyunlee@10.42.0.1', self.messages.getvalue())
        self.assertIn('현재 SSH 접속: ssh hyunlee@192.168.0.37', self.messages.getvalue())
        self.assertIn('12345678', self.messages.getvalue())

    def test_boot_only_does_not_schedule_or_activate(self):
        with redirect_stdout(self.messages):
            hotspot.enable(boot_only=True)
        self.assertFalse(any(c[0] == 'systemd-run' or 'up' in c for c in self.calls))

    def test_disable_removes_boot_autoconnect_without_dropping_ssh(self):
        with patch.object(Path, 'exists', return_value=True), redirect_stdout(self.messages):
            hotspot.disable()
        self.assertIn(('systemctl', 'disable', '--now', hotspot.SERVICE), self.calls)
        self.assertIn(('nmcli', 'connection', 'modify', 'uuid', 'ap-uuid', 'connection.autoconnect', 'no'), self.calls)
        self.assertFalse(any('down' in c or 'disconnect' in c or c[0] == 'systemd-run' for c in self.calls))

    def test_boot_activation_and_disabled_guard(self):
        hotspot.boot()
        self.assertIn(('nmcli', '--wait', '45', 'connection', 'up', 'uuid', 'ap-uuid', 'ifname', hotspot.DEVICE), self.calls)
        self.calls.clear()
        with patch.object(hotspot, 'value', return_value='no'):
            hotspot.boot()
        self.assertEqual(self.calls, [])

    def test_unsupported_adapter_does_not_mutate(self):
        with patch.object(hotspot, 'value', return_value='no'):
            with self.assertRaises(RuntimeError):
                hotspot.enable()
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main()
