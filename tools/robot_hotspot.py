#!/usr/bin/env python3
"""Configure this robot's NetworkManager hotspot and persistent boot service."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

DEVICE = 'wlP1p1s0'
PROFILE = 'orinbot-outdoor'
SSID = 'ORINBOT'
PASSWORD = '12345678'
ADDRESS = '10.42.0.1'
SSH_USER = 'hyunlee'
SERVICE = 'robot-hotspot.service'
TIMER = 'robot-hotspot-activate'
INSTALLED = Path('/usr/local/lib/robot-hotspot/robot_hotspot.py')
UNIT_FILE = Path('/etc/systemd/system') / SERVICE


def run(*args, check=True):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            env={**os.environ, 'LC_ALL': 'C'}, timeout=65)
    if check and result.returncode:
        # Don't repeat command arguments: they can include a Wi-Fi secret.
        raise RuntimeError(f'{args[0]} 실패: {result.stderr.strip()}')
    return result


def value(field, *target, secrets=False):
    args = ['nmcli', '--escape', 'no', '--get-values', field]
    if secrets:
        args.append('--show-secrets')
    result = run(*args, *target, check=False)
    return result.stdout.strip() if result.returncode == 0 else ''


def profile_uuid():
    return value('connection.uuid', 'connection', 'show', PROFILE)


def show_current():
    uuid = value('GENERAL.CON-UUID', 'device', 'show', DEVICE)
    if not uuid or uuid == '--':
        print('현재 Wi-Fi: 연결되지 않음 / SSH용 Wi-Fi IPv4 주소 없음')
        return
    ssid = value('802-11-wireless.ssid', 'connection', 'show', 'uuid', uuid)
    password = value('802-11-wireless-security.psk', 'connection', 'show', 'uuid', uuid, secrets=True)
    addresses = value('IP4.ADDRESS', 'device', 'show', DEVICE).splitlines()
    print(f'현재 Wi-Fi 이름: {ssid or "확인 불가"}')
    print(f'현재 Wi-Fi 저장 비밀번호: {password or "저장된 PSK를 읽을 수 없음 (sudo로 확인)"}')
    for address in addresses:
        ip = address.split('/')[0]
        if ip and ip != '--':
            print(f'현재 SSH 접속: ssh {SSH_USER}@{ip}')
    if not addresses:
        print('현재 Wi-Fi IPv4 주소: 없음')


def show_hotspot():
    print(f'핫스팟 Wi-Fi 이름: {SSID}')
    print(f'핫스팟 비밀번호: {PASSWORD}')
    print(f'핫스팟 SSH 접속: ssh {SSH_USER}@{ADDRESS}')
    print('위 비밀번호는 Wi-Fi용입니다. SSH 로그인에는 기존 계정 인증을 사용합니다.')


def unit_text():
    return f'''[Unit]
Description=Robot Wi-Fi hotspot at boot
Wants=NetworkManager.service
After=NetworkManager.service
StartLimitIntervalSec=0

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 {INSTALLED} boot
RemainAfterExit=yes
Restart=on-failure
RestartSec=15
TimeoutStartSec=70

[Install]
WantedBy=multi-user.target
'''


def install_service():
    INSTALLED.parent.mkdir(parents=True, exist_ok=True)
    if Path(__file__).resolve() != INSTALLED:
        shutil.copyfile(__file__, INSTALLED)
    INSTALLED.chmod(0o755)
    UNIT_FILE.write_text(unit_text())
    UNIT_FILE.chmod(0o644)
    run('systemctl', 'daemon-reload')


def cancel_pending():
    run('systemctl', 'stop', TIMER + '.timer', TIMER + '.service', check=False)
    run('systemctl', 'reset-failed', TIMER + '.timer', TIMER + '.service', check=False)


def enable(boot_only=False):
    if value('WIFI-PROPERTIES.AP', 'device', 'show', DEVICE) != 'yes':
        raise RuntimeError(f'{DEVICE}에서 AP 지원을 확인할 수 없습니다.')
    if run('systemctl', 'is-active', '--quiet', 'orinbot-wifi-switch.service', check=False).returncode == 0:
        raise RuntimeError('기존 orinbot-wifi 전환 작업이 진행 중입니다. 완료 후 다시 실행하세요.')
    uuid = profile_uuid()
    if not uuid:
        run('nmcli', 'connection', 'add', 'type', 'wifi', 'ifname', DEVICE,
            'con-name', PROFILE, 'ssid', SSID, 'connection.autoconnect', 'no')
        uuid = profile_uuid()
    if not uuid:
        raise RuntimeError('핫스팟 프로필 UUID를 확인할 수 없습니다.')
    run('nmcli', 'connection', 'modify', 'uuid', uuid,
        'connection.interface-name', DEVICE, 'connection.permissions', '',
        'connection.autoconnect', 'yes', 'connection.autoconnect-priority', '999',
        'connection.autoconnect-retries', '0',
        '802-11-wireless.mode', 'ap', '802-11-wireless.ssid', SSID,
        '802-11-wireless.band', 'bg', '802-11-wireless.channel', '6',
        '802-11-wireless.powersave', '2',
        '802-11-wireless-security.key-mgmt', 'wpa-psk',
        '802-11-wireless-security.proto', 'rsn',
        '802-11-wireless-security.pairwise', 'ccmp',
        '802-11-wireless-security.group', 'ccmp',
        '802-11-wireless-security.psk-flags', '0',
        '802-11-wireless-security.psk', PASSWORD,
        'ipv4.method', 'shared', 'ipv4.addresses', ADDRESS + '/24',
        'ipv4.never-default', 'yes', 'ipv6.method', 'disabled')
    install_service()
    run('systemctl', 'enable', '--now', 'ssh.service')
    run('systemctl', 'enable', SERVICE)
    cancel_pending()
    print('부팅 시 핫스팟 자동 시작: 켜짐')
    show_current()
    show_hotspot()
    if boot_only:
        print('핫스팟 전환은 다음 부팅에 적용됩니다.')
    else:
        print('5초 후 핫스팟으로 전환합니다. 현재 Wi-Fi SSH 연결이 끊기면 ORINBOT에 연결한 뒤 위 주소로 재접속하세요.', flush=True)
        run('systemd-run', '--quiet', '--collect', '--unit=' + TIMER,
            '--on-active=5s', '--timer-property=AccuracySec=1s',
            '/usr/bin/systemctl', 'restart', SERVICE)


def disable():
    # Disable persistence only: keep the active AP so this SSH session survives.
    if run('systemctl', 'is-active', '--quiet', 'orinbot-wifi-switch.service', check=False).returncode == 0:
        raise RuntimeError('기존 orinbot-wifi 전환 작업이 진행 중입니다. 완료 후 다시 실행하세요.')
    cancel_pending()
    if UNIT_FILE.exists():
        run('systemctl', 'disable', '--now', SERVICE)
    uuid = profile_uuid()
    if uuid:
        run('nmcli', 'connection', 'modify', 'uuid', uuid, 'connection.autoconnect', 'no')
    print('부팅 시 핫스팟 자동 시작: 꺼짐')
    print('현재 Wi-Fi 연결은 유지됩니다. 다음 부팅에는 저장된 일반 Wi-Fi로 자동 연결을 시도합니다.')
    show_current()


def boot():
    uuid = profile_uuid()
    if not uuid:
        raise RuntimeError('핫스팟 프로필이 없습니다.')
    if value('connection.autoconnect', 'connection', 'show', 'uuid', uuid) != 'yes':
        return
    run('nmcli', 'radio', 'wifi', 'on')
    run('nmcli', '--wait', '45', 'connection', 'up', 'uuid', uuid, 'ifname', DEVICE)


def main(argv=None):
    parser = argparse.ArgumentParser(description='로봇 핫스팟 부팅 자동 시작 설정 및 Wi-Fi/SSH 정보 표시')
    parser.add_argument('action', choices=['enable', 'disable', 'boot'])
    parser.add_argument('--boot-only', action='store_true', help='켜기: 지금 전환하지 않고 다음 부팅부터 적용')
    parser.add_argument('--status', action='store_true', help='설정 변경 없이 현재 Wi-Fi 정보 표시 (비밀번호 확인은 sudo)')
    args = parser.parse_args(argv)
    if args.boot_only and args.action != 'enable':
        parser.error('--boot-only는 켜기 스크립트에만 사용할 수 있습니다.')
    if not args.status and os.geteuid() != 0:
        print('네트워크 및 부팅 설정을 변경하려면 sudo 인증이 필요합니다.', flush=True)
        os.execvp('sudo', ['sudo', sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]])
    try:
        if args.status:
            uuid = profile_uuid()
            auto = value('connection.autoconnect', 'connection', 'show', 'uuid', uuid) if uuid else 'no'
            print(f'핫스팟 프로필 자동 연결: {auto}')
            show_current()
        elif args.action == 'enable':
            enable(args.boot_only)
        elif args.action == 'disable':
            disable()
        else:
            boot()
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print(f'핫스팟 설정 오류: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
