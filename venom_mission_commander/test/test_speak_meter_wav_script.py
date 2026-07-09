"""Tests for the meter WAV speaker helper script."""

import os
import subprocess
from pathlib import Path


WORKSPACE_DIR = Path(__file__).resolve().parents[4]
SCRIPT_PATH = WORKSPACE_DIR / 'scripts' / 'speak_meter_wav.sh'
USB_ANALOG_SINK = (
    'alsa_output.usb-KTMicro_KT_USB_Audio_2021-06-07-0000-0000-0000--00.'
    'analog-stereo'
)


def write_executable(path, content):
    path.write_text(content, encoding='utf-8')
    path.chmod(0o755)


def test_speak_meter_wav_prefers_available_usb_analog_sink(tmp_path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    calls_path = tmp_path / 'paplay_calls.txt'

    write_executable(
        fake_bin / 'pactl',
        f"""#!/usr/bin/env bash
set -euo pipefail
if [ "$*" = "list short sinks" ]; then
  printf '5\t{USB_ANALOG_SINK}\tmodule-alsa-card.c\ts16le 2ch 44100Hz\tIDLE\n'
elif [ "$*" = "get-default-sink" ]; then
  printf 'alsa_output.pci-0000_00_1f.3.hdmi-stereo\n'
else
  exit 2
fi
""",
    )
    write_executable(
        fake_bin / 'paplay',
        """#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >> "$PAPLAY_CALLS"
""",
    )

    env = os.environ.copy()
    env['PATH'] = f"{fake_bin}:{env['PATH']}"
    env['PLAYER'] = 'paplay'
    env['PAPLAY_CALLS'] = str(calls_path)
    env.pop('PULSE_SINK', None)

    subprocess.run(
        [str(SCRIPT_PATH), 'meter reading 1234'],
        check=True,
        env=env,
        text=True,
    )

    calls = calls_path.read_text(encoding='utf-8').splitlines()
    assert calls
    assert all(call.startswith(f'--device={USB_ANALOG_SINK} ') for call in calls)
