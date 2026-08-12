"""Preset IDs used by the serial voice-module command interface."""

from __future__ import annotations

from enum import IntEnum


class VoiceId(IntEnum):
    """Common preset IDs from Yahboom ROS voice firmware."""

    STOP = 2
    GO_AHEAD = 4
    BACK = 5
    TURN_LEFT = 6
    TURN_RIGHT = 7
    MODE_A = 8
    MODE_B = 9
    LIGHT_OFF = 10
    LIGHT_RED = 11
    LIGHT_GREEN = 12
    LIGHT_BLUE = 13
    LIGHT_YELLOW = 14
    NAV_POINT_1 = 19
    NAV_POINT_2 = 20
    NAV_POINT_3 = 21
    NAV_POINT_4 = 32
    RETURN_ORIGIN = 33
    CAR_STOPPED = 0x11
    CAR_FORWARD = 0x03
    CAR_BACK = 0x05
    CAR_TURN_LEFT = 0x0F
    CAR_TURN_RIGHT = 0x10


VOICE_ID_NAMES: dict[int, str] = {
    VoiceId.STOP: 'stop',
    VoiceId.GO_AHEAD: 'go_ahead',
    VoiceId.BACK: 'back',
    VoiceId.TURN_LEFT: 'turn_left',
    VoiceId.TURN_RIGHT: 'turn_right',
    VoiceId.MODE_A: 'mode_a',
    VoiceId.MODE_B: 'mode_b',
    VoiceId.LIGHT_OFF: 'light_off',
    VoiceId.LIGHT_RED: 'light_red',
    VoiceId.LIGHT_GREEN: 'light_green',
    VoiceId.LIGHT_BLUE: 'light_blue',
    VoiceId.LIGHT_YELLOW: 'light_yellow',
    VoiceId.NAV_POINT_1: 'nav_point_1',
    VoiceId.NAV_POINT_2: 'nav_point_2',
    VoiceId.NAV_POINT_3: 'nav_point_3',
    VoiceId.NAV_POINT_4: 'nav_point_4',
    VoiceId.RETURN_ORIGIN: 'return_origin',
    VoiceId.CAR_STOPPED: 'car_stopped',
    VoiceId.CAR_FORWARD: 'car_forward',
    VoiceId.CAR_BACK: 'car_back',
    VoiceId.CAR_TURN_LEFT: 'car_turn_left',
    VoiceId.CAR_TURN_RIGHT: 'car_turn_right',
}

VOICE_NAME_TO_ID: dict[str, int] = {name: int(vid) for vid, name in VOICE_ID_NAMES.items()}


def voice_name_for_id(voice_id: int) -> str:
    return VOICE_ID_NAMES.get(int(voice_id), f'voice_{int(voice_id)}')


def voice_id_for_name(name: str) -> int | None:
    key = str(name).strip().lower()
    if key.isdigit():
        return int(key)
    return VOICE_NAME_TO_ID.get(key)
