from __future__ import annotations

import argparse
import datetime
import math
import select
import signal
import sys
import threading
import time
from dataclasses import dataclass

import serial
from evdev import AbsInfo, UInput
from evdev import ecodes as e

BRIDGE_START = 0xAB
PKT_TELEMETRY = 0x10
PKT_TORQUE = 0x20
PKT_CONTROL = 0x30

TELEMETRY_LEN = 18
TORQUE_LEN = 8
CONTROL_LEN = 7

CMD_STARTUP = 0x01
STARTUP_FLAG_AUTO_CALIB = 0x0001


@dataclass
class EffectEnvelope:
	attack_length_ms: int
	attack_level: int
	fade_length_ms: int
	fade_level: int


class BridgeLogger:
	def __init__(self, path: str | None) -> None:
		self.path = path
		self._lock = threading.Lock()
		self._file = None
		if path:
			self._file = open(path, "a", encoding="utf-8", buffering=1)
			self.log("[LOG] ---- bridge session started ----")

	def log(self, message: str) -> None:
		if not self._file:
			return
		ts = datetime.datetime.now().isoformat(timespec="milliseconds")
		with self._lock:
			self._file.write(f"{ts} {message}\n")

	def close(self) -> None:
		if not self._file:
			return
		with self._lock:
			try:
				self._file.write(
					f"{datetime.datetime.now().isoformat(timespec='milliseconds')} [LOG] ---- bridge session ended ----\n"
				)
				self._file.flush()
			finally:
				self._file.close()
				self._file = None


class TorqueState:
	def __init__(self, max_torque: int, min_torque: int = 0, use_min_torque_comp: bool = False, input_deadzone: int = 0, min_torque_input: int = 0, torque_response_gamma: float = 1.0) -> None:
		self.max_torque = max(1, int(max_torque))
		self.min_torque = max(0, int(min_torque))
		self.use_min_torque_comp = bool(use_min_torque_comp)
		self.input_deadzone = max(0, int(input_deadzone))
		self.min_torque_input = max(0, int(min_torque_input))
		self.torque_response_gamma = max(0.1, float(torque_response_gamma))
		self.device_gain = 255
		self._effect_contribution: dict[int, float] = {}
		self._effect_active: dict[int, bool] = {}
		self.x_norm = 0.0
		self.x_vel = 0.0
		self.x_acc = 0.0
		self._last_motion_ts = 0.0
		self._lock = threading.Lock()

	def set_device_gain(self, gain: int) -> None:
		with self._lock:
			self.device_gain = max(0, min(255, int(gain)))

	def set_effect_contribution(self, effect_id: int, contribution: float) -> None:
		with self._lock:
			self._effect_contribution[int(effect_id)] = max(-1.0, min(1.0, float(contribution)))

	def set_effect_active(self, effect_id: int, active: bool) -> None:
		with self._lock:
			self._effect_active[int(effect_id)] = bool(active)

	def stop_effect(self, effect_id: int) -> None:
		with self._lock:
			self._effect_active[int(effect_id)] = False
			self._effect_contribution[int(effect_id)] = 0.0

	def clear_all_effects(self) -> None:
		with self._lock:
			self._effect_active.clear()
			self._effect_contribution.clear()

	def update_motion(self, x_raw: int, now: float | None = None) -> None:
		now_ts = time.monotonic() if now is None else float(now)
		x = max(-32768, min(32767, int(x_raw)))
		x_norm = x / 32767.0 if x >= 0 else x / 32768.0
		with self._lock:
			if self._last_motion_ts > 0.0:
				dt = max(1e-6, now_ts - self._last_motion_ts)
				new_vel = (x_norm - self.x_norm) / dt
				self.x_acc = (new_vel - self.x_vel) / dt
				self.x_vel = new_vel
			self.x_norm = x_norm
			self._last_motion_ts = now_ts

	def snapshot(self) -> tuple[int, dict[int, float], dict[int, bool], dict[str, float]]:
		with self._lock:
			return (
				int(self.device_gain),
				dict(self._effect_contribution),
				dict(self._effect_active),
				{"x_norm": self.x_norm, "x_vel": self.x_vel, "x_acc": self.x_acc},
			)

	def compute_torque(self) -> int:
		with self._lock:
			summed = 0.0
			for effect_id, contribution in self._effect_contribution.items():
				if not self._effect_active.get(effect_id, False):
					continue
				summed += contribution
			summed = max(-1.0, min(1.0, summed))
			scaled = summed * self.max_torque
			scaled *= self.device_gain / 255.0
		torque = int(round(scaled))
		torque = -torque
		if torque == 0:
			return 0
		sign = 1 if torque > 0 else -1
		mag = max(0, min(self.max_torque, abs(int(torque))))
		if mag <= self.input_deadzone:
			return 0
		if self.use_min_torque_comp and self.min_torque > 0 and self.min_torque < self.max_torque:
			in_start = max(0, min(self.max_torque, self.min_torque_input))
			if mag <= in_start:
				mag = self.min_torque
			else:
				in_span = max(1, self.max_torque - in_start)
				norm = max(0.0, min(1.0, (mag - in_start) / in_span))
				norm = norm**self.torque_response_gamma
				out_span = self.max_torque - self.min_torque
				mag = int(round(self.min_torque + (norm * out_span)))
		return sign * max(0, min(self.max_torque, mag))


class FFBEffectsManagerLinux:
	MAX_TORQUE = 300
	USE_MIN_TORQUE_COMP = True
	MIN_TORQUE = 160
	INPUT_DEADZONE = 5
	MIN_TORQUE_INPUT = 0
	TORQUE_RESPONSE_GAMMA = 1.0

	GLOBAL_FORCE_SCALE = 1.0
	PERIODIC_FORCE_SCALE = 1.0
	CONDITION_FORCE_SCALE = 1.0
	CONSTANT_OVERLAY_SCALE = 1.0

	CONST_FORCE_SCALE = 1.0
	RAMP_FORCE_SCALE = 0.5
	SINE_FORCE_SCALE = 0.5
	SQUARE_FORCE_SCALE = 0.5
	TRIANGLE_FORCE_SCALE = 0.5
	SAW_UP_FORCE_SCALE = 0.5
	SAW_DOWN_FORCE_SCALE = 0.5
	SPRING_FORCE_SCALE = 0.5
	DAMPER_FORCE_SCALE = 0.5
	INERTIA_FORCE_SCALE = 0.5
	FRICTION_FORCE_SCALE = 0.5
	RUMBLE_FORCE_SCALE = 0.2

	def __init__(self, torque_state: TorqueState, logger: BridgeLogger | None = None, debug_ffb: bool = False, debug_print_interval: float = 0.05) -> None:
		self.torque_state = torque_state
		self.logger = logger
		self.debug_ffb = debug_ffb
		self.debug_print_interval = max(0.0, float(debug_print_interval))
		self._last_debug_ts = 0.0
		self._effects: dict[int, object] = {}
		self._active: dict[int, bool] = {}
		self._start_ts: dict[int, float] = {}
		self._play_repeats: dict[int, int] = {}

	def _log(self, msg: str) -> None:
		if self.logger is not None:
			self.logger.log(f"[FFB] {msg}")
		if not self.debug_ffb:
			return
		now = time.monotonic()
		if now - self._last_debug_ts < self.debug_print_interval:
			return
		self._last_debug_ts = now
		print(f"[FFB] {msg}", flush=True)

	@staticmethod
	def _i16_norm(value: int) -> float:
		if value <= -32768:
			return -1.0
		return max(-1.0, min(1.0, int(value) / 32767.0))

	@staticmethod
	def _u16_norm(value: int) -> float:
		return max(0.0, min(1.0, int(value) / 65535.0))

	@staticmethod
	def _phase_rad(phase_u16: int) -> float:
		return (2.0 * math.pi) * (max(0, min(65535, int(phase_u16))) / 65535.0)

	@staticmethod
	def _direction_x_scale(direction_u16: int) -> float:
		# Linux ff_effect.direction uses centidegrees (0..35999), not a raw 0..65535 angle.
		angle = math.radians((int(direction_u16) % 36000) / 100.0)
		return -math.cos(angle)

	@staticmethod
	def _compute_periodic_wave(waveform: int, phase: float) -> float:
		two_pi = 2.0 * math.pi
		p = phase % two_pi
		u = p / two_pi
		if waveform == e.FF_SINE:
			return math.sin(p)
		if waveform == e.FF_SQUARE:
			return 1.0 if math.sin(p) >= 0.0 else -1.0
		if waveform == e.FF_TRIANGLE:
			return 2.0 * abs(2.0 * (u - math.floor(u + 0.5))) - 1.0
		if waveform == e.FF_SAW_UP:
			return 2.0 * u - 1.0
		if waveform == e.FF_SAW_DOWN:
			return 1.0 - 2.0 * u
		return 0.0

	@staticmethod
	def _envelope_from_effect(effect: object) -> EffectEnvelope | None:
		if effect.type == e.FF_CONSTANT:
			env = effect.u.ff_constant_effect.ff_envelope
			return EffectEnvelope(
				attack_length_ms=int(env.attack_length),
				attack_level=int(env.attack_level),
				fade_length_ms=int(env.fade_length),
				fade_level=int(env.fade_level),
			)
		if effect.type == e.FF_RAMP:
			env = effect.u.ff_ramp_effect.ff_envelope
			return EffectEnvelope(
				attack_length_ms=int(env.attack_length),
				attack_level=int(env.attack_level),
				fade_length_ms=int(env.fade_length),
				fade_level=int(env.fade_level),
			)
		if effect.type == e.FF_PERIODIC:
			env = effect.u.ff_periodic_effect.envelope
			return EffectEnvelope(
				attack_length_ms=int(env.attack_length),
				attack_level=int(env.attack_level),
				fade_length_ms=int(env.fade_length),
				fade_level=int(env.fade_level),
			)
		return None

	def _envelope_scale(self, effect: object, now: float, start_ts: float, local_t_ms: float, duration_ms: int) -> float:
		env = self._envelope_from_effect(effect)
		if env is None:
			return 1.0
		out = 1.0
		attack_level = self._u16_norm(env.attack_level)
		fade_level = self._u16_norm(env.fade_level)
		if env.attack_length_ms > 0 and local_t_ms < env.attack_length_ms:
			t = local_t_ms / float(env.attack_length_ms)
			out *= attack_level + (1.0 - attack_level) * t
		if duration_ms > 0 and env.fade_length_ms > 0:
			total_elapsed_ms = max(0.0, (now - start_ts) * 1000.0)
			remaining = duration_ms - total_elapsed_ms
			if remaining <= env.fade_length_ms:
				t = max(0.0, remaining / float(env.fade_length_ms))
				out *= fade_level + (1.0 - fade_level) * t
		return max(0.0, min(1.0, out))

	@staticmethod
	def _condition_force(cond: object, signal_value: float) -> float:
		center = FFBEffectsManagerLinux._i16_norm(cond.center)
		deadband = FFBEffectsManagerLinux._u16_norm(cond.deadband)
		pos_coeff = FFBEffectsManagerLinux._i16_norm(cond.right_coeff)
		neg_coeff = FFBEffectsManagerLinux._i16_norm(cond.left_coeff)
		pos_sat = FFBEffectsManagerLinux._u16_norm(cond.right_saturation)
		neg_sat = FFBEffectsManagerLinux._u16_norm(cond.left_saturation)
		err = signal_value - center
		if abs(err) <= deadband:
			return 0.0
		if err > 0.0:
			return max(-pos_sat, min(pos_sat, err * pos_coeff))
		return max(-neg_sat, min(neg_sat, err * neg_coeff))

	def register_upload(self, effect: object) -> None:
		effect_id = int(effect.id)
		self._effects[effect_id] = effect
		if effect_id not in self._active:
			self._active[effect_id] = False
		self._log(f"upload id={effect_id} type={int(effect.type)}")

	def register_erase(self, effect_id: int) -> None:
		eid = int(effect_id)
		self._effects.pop(eid, None)
		self._active.pop(eid, None)
		self._start_ts.pop(eid, None)
		self._play_repeats.pop(eid, None)
		self.torque_state.stop_effect(eid)
		self._log(f"erase id={eid}")

	def register_play(self, effect_id: int, value: int, now: float) -> None:
		eid = int(effect_id)
		val = int(value)
		if val <= 0:
			self._active[eid] = False
			self._start_ts.pop(eid, None)
			self._play_repeats.pop(eid, None)
			self.torque_state.stop_effect(eid)
			self._log(f"stop id={eid}")
			return
		self._active[eid] = True
		self._start_ts[eid] = now
		self._play_repeats[eid] = val
		self.torque_state.set_effect_active(eid, True)
		self._log(f"play id={eid} loops={val}")

	def register_gain(self, ff_gain_u16: int) -> None:
		gain = int(round(max(0, min(65535, int(ff_gain_u16))) * 255.0 / 65535.0))
		self.torque_state.set_device_gain(gain)
		self._log(f"gain {gain}/255")

	def stop_all(self) -> None:
		for eid in list(self._active.keys()):
			self._active[eid] = False
			self.torque_state.stop_effect(eid)

	def tick(self, now: float) -> None:
		for eid, is_active in list(self._active.items()):
			if not is_active:
				self.torque_state.set_effect_contribution(eid, 0.0)
				continue

			effect = self._effects.get(eid)
			if effect is None:
				self._active[eid] = False
				self.torque_state.stop_effect(eid)
				continue

			start_ts = self._start_ts.get(eid, now)
			elapsed_ms = max(0.0, (now - start_ts) * 1000.0)
			replay_delay_ms = int(effect.ff_replay.delay)
			replay_len_ms = int(effect.ff_replay.length)
			repeats = max(1, int(self._play_repeats.get(eid, 1)))

			if elapsed_ms < replay_delay_ms:
				self.torque_state.set_effect_contribution(eid, 0.0)
				continue

			active_ms = elapsed_ms - replay_delay_ms
			duration_ms = replay_len_ms if replay_len_ms > 0 else 0
			if duration_ms > 0 and active_ms > (duration_ms * repeats):
				self._active[eid] = False
				self.torque_state.stop_effect(eid)
				continue

			local_t_ms = active_ms
			if duration_ms > 0:
				local_t_ms = active_ms % duration_ms

			x_scale = self._direction_x_scale(int(effect.direction))
			force = 0.0

			if effect.type == e.FF_CONSTANT:
				const = effect.u.ff_constant_effect
				force = self._i16_norm(const.level) * x_scale * self.CONST_FORCE_SCALE

			elif effect.type == e.FF_RAMP:
				ramp = effect.u.ff_ramp_effect
				if duration_ms <= 0:
					mag = self._i16_norm(ramp.end_level)
				else:
					t = max(0.0, min(1.0, local_t_ms / float(duration_ms)))
					start = self._i16_norm(ramp.start_level)
					end = self._i16_norm(ramp.end_level)
					mag = start + ((end - start) * t)
				force = mag * x_scale * self.RAMP_FORCE_SCALE

			elif effect.type == e.FF_PERIODIC:
				per = effect.u.ff_periodic_effect
				wave = int(per.waveform)
				period_ms = max(1.0, float(per.period))
				phase = (2.0 * math.pi) * (local_t_ms / period_ms) + self._phase_rad(int(per.phase))
				wave_val = self._compute_periodic_wave(wave, phase)
				mag = self._i16_norm(per.magnitude)
				offs = self._i16_norm(per.offset)
				wave_scale = self.PERIODIC_FORCE_SCALE
				if wave == e.FF_SINE:
					wave_scale *= self.SINE_FORCE_SCALE
				elif wave == e.FF_SQUARE:
					wave_scale *= self.SQUARE_FORCE_SCALE
				elif wave == e.FF_TRIANGLE:
					wave_scale *= self.TRIANGLE_FORCE_SCALE
				elif wave == e.FF_SAW_UP:
					wave_scale *= self.SAW_UP_FORCE_SCALE
				elif wave == e.FF_SAW_DOWN:
					wave_scale *= self.SAW_DOWN_FORCE_SCALE
				force = (offs + (wave_val * mag)) * x_scale * wave_scale

			elif effect.type == e.FF_SPRING:
				cond = effect.u.ff_condition_effect[0]
				force = (
					self._condition_force(cond, self.torque_state.x_norm)
					* self.CONDITION_FORCE_SCALE
					* self.SPRING_FORCE_SCALE
				)

			elif effect.type == e.FF_DAMPER:
				cond = effect.u.ff_condition_effect[0]
				force = (
					self._condition_force(cond, self.torque_state.x_vel)
					* self.CONDITION_FORCE_SCALE
					* self.DAMPER_FORCE_SCALE
				)

			elif effect.type == e.FF_INERTIA:
				cond = effect.u.ff_condition_effect[0]
				force = (
					self._condition_force(cond, self.torque_state.x_acc)
					* self.CONDITION_FORCE_SCALE
					* self.INERTIA_FORCE_SCALE
				)

			elif effect.type == e.FF_FRICTION:
				cond = effect.u.ff_condition_effect[0]
				signal = 0.0
				if self.torque_state.x_vel > 0.0:
					signal = 1.0
				elif self.torque_state.x_vel < 0.0:
					signal = -1.0
				force = (
					self._condition_force(cond, signal)
					* self.CONDITION_FORCE_SCALE
					* self.FRICTION_FORCE_SCALE
				)

			elif effect.type == e.FF_RUMBLE:
				rumble = effect.u.ff_rumble_effect
				strong = self._u16_norm(rumble.strong_magnitude)
				weak = self._u16_norm(rumble.weak_magnitude)
				force = ((strong * 0.7) + (weak * 0.3)) * self.RUMBLE_FORCE_SCALE

			env_scale = self._envelope_scale(effect, now, start_ts, local_t_ms, duration_ms)
			force *= env_scale
			force *= self.GLOBAL_FORCE_SCALE
			force = max(-1.0, min(1.0, force))
			self.torque_state.set_effect_contribution(eid, force)


class LinuxFFBBridge:
	def __init__(
		self,
		torque_state: TorqueState,
		logger: BridgeLogger | None = None,
		debug_ffb: bool = False,
		debug_print_interval: float = 0.05,
		max_effects: int = 96,
	) -> None:
		self.torque_state = torque_state
		self.logger = logger
		self.max_effects = max(1, int(max_effects))
		self.effects = FFBEffectsManagerLinux(
			torque_state,
			logger=logger,
			debug_ffb=debug_ffb,
			debug_print_interval=debug_print_interval,
		)
		self.ui: UInput | None = None
		self.button_codes = [e.BTN_TRIGGER_HAPPY1 + i for i in range(32)]
		self._dpad_codes = (e.BTN_DPAD_UP, e.BTN_DPAD_LEFT, e.BTN_DPAD_RIGHT, e.BTN_DPAD_DOWN)
		self._last_axes = (None, None, None)
		self._last_buttons = -1
		self._last_dpad = (None, None, None, None)
		self._last_hat = (None, None)
		self._ffb_stop_event = threading.Event()
		self._ffb_thread: threading.Thread | None = None

	def _build_capabilities(self) -> dict[int, list[object]]:
		abs_x = (e.ABS_X, AbsInfo(0, -32768, 32767, 0, 0, 0))
		abs_y = (e.ABS_Y, AbsInfo(0, -32768, 32767, 0, 0, 0))
		abs_z = (e.ABS_Z, AbsInfo(0, -32768, 32767, 0, 0, 0))
		abs_gas = (e.ABS_GAS, AbsInfo(0, 0, 32767, 0, 0, 0))
		abs_brake = (e.ABS_BRAKE, AbsInfo(0, 0, 32767, 0, 0, 0))
		hat_x = (e.ABS_HAT0X, AbsInfo(0, -1, 1, 0, 0, 0))
		hat_y = (e.ABS_HAT0Y, AbsInfo(0, -1, 1, 0, 0, 0))

		ff_caps = [
			e.FF_CONSTANT,
			e.FF_RAMP,
			e.FF_PERIODIC,
			e.FF_SQUARE,
			e.FF_TRIANGLE,
			e.FF_SINE,
			e.FF_SAW_UP,
			e.FF_SAW_DOWN,
			e.FF_SPRING,
			e.FF_DAMPER,
			e.FF_INERTIA,
			e.FF_FRICTION,
			e.FF_RUMBLE,
			e.FF_GAIN,
			e.FF_AUTOCENTER,
		]

		return {
			e.EV_KEY: self.button_codes,
			e.EV_ABS: [abs_x, abs_y, abs_z, abs_gas, abs_brake, hat_x, hat_y],
			e.EV_FF: ff_caps,
		}

	def start(self, name: str, vendor: int, product: int, version: int) -> None:
		caps = self._build_capabilities()
		self.ui = UInput(
			events=caps,
			name=name,
			vendor=vendor,
			product=product,
			version=version,
			bustype=e.BUS_USB,
			max_effects=self.max_effects,
		)
		if self.logger and self.logger.path:
			dev_path = self.ui.device.path if self.ui.device is not None else "unknown"
			self.logger.log(f"[UINPUT] created name={name} dev={dev_path}")
		self._ffb_stop_event.clear()
		self._ffb_thread = threading.Thread(target=self._ffb_worker, daemon=True)
		self._ffb_thread.start()

	def stop(self) -> None:
		self._ffb_stop_event.set()
		if self._ffb_thread is not None:
			self._ffb_thread.join(timeout=1.0)
			self._ffb_thread = None
		self.effects.stop_all()
		if self.ui is not None:
			try:
				self.ui.close()
			except Exception:
				pass
			self.ui = None

	@staticmethod
	def _pedal_value(raw: int) -> int:
		return max(0, min(32767, int(round(abs(int(raw)) * 32767 / 32768))))

	def update_axes_buttons(self, x: int, y: int, z: int, buttons: int) -> None:
		if self.ui is None:
			return

		x = max(-32768, min(32767, int(x)))
		y = max(-32768, min(32767, int(y)))
		z = max(-32768, min(32767, int(z)))

		if (x, y, z) != self._last_axes:
			self.ui.write(e.EV_ABS, e.ABS_X, x)
			self.ui.write(e.EV_ABS, e.ABS_Y, y)
			self.ui.write(e.EV_ABS, e.ABS_Z, z)
			self.ui.write(e.EV_ABS, e.ABS_GAS, self._pedal_value(y))
			self.ui.write(e.EV_ABS, e.ABS_BRAKE, self._pedal_value(z))
			self._last_axes = (x, y, z)

		if buttons != self._last_buttons:
			for i, code in enumerate(self.button_codes):
				pressed = 1 if (buttons & (1 << i)) else 0
				self.ui.write(e.EV_KEY, code, pressed)
			self._last_buttons = buttons

		up, left, right, down = (
			1 if (buttons & (1 << 32)) else 0,
			1 if (buttons & (1 << 33)) else 0,
			1 if (buttons & (1 << 34)) else 0,
			1 if (buttons & (1 << 35)) else 0,
		)
		hx = (-1 if left else 0) + (1 if right else 0)
		hy = (-1 if up else 0) + (1 if down else 0)
		hx = max(-1, min(1, hx))
		hy = max(-1, min(1, hy))
		if (hx, hy) != self._last_hat:
			self.ui.write(e.EV_ABS, e.ABS_HAT0X, hx)
			self.ui.write(e.EV_ABS, e.ABS_HAT0Y, hy)
			self._last_hat = (hx, hy)
		if (up, left, right, down) != self._last_dpad:
			self.ui.write(e.EV_KEY, self._dpad_codes[0], up)
			self.ui.write(e.EV_KEY, self._dpad_codes[1], left)
			self.ui.write(e.EV_KEY, self._dpad_codes[2], right)
			self.ui.write(e.EV_KEY, self._dpad_codes[3], down)
			self._last_dpad = (up, left, right, down)

		self.ui.syn()

	def process_ff_events(self, now: float) -> None:
		if self.ui is None:
			return
		try:
			# FF upload/play events are read from the uinput fd, not /dev/input/eventX.
			for event in self.ui.read():
				if event.type == e.EV_UINPUT:
					if event.code == e.UI_FF_UPLOAD:
						upload = self.ui.begin_upload(event.value)
						self.effects.register_upload(upload.effect)
						upload.retval = 0
						self.ui.end_upload(upload)
					elif event.code == e.UI_FF_ERASE:
						erase = self.ui.begin_erase(event.value)
						self.effects.register_erase(int(erase.effect_id))
						erase.retval = 0
						self.ui.end_erase(erase)
				elif event.type == e.EV_FF:
					if event.code == e.FF_GAIN:
						self.effects.register_gain(event.value)
					elif event.code == e.FF_AUTOCENTER:
						# Autocenter can be implemented in firmware if needed.
						pass
					else:
						self.effects.register_play(event.code, event.value, now)
		except BlockingIOError:
			return
		except OSError:
			return

	def _ffb_worker(self) -> None:
		while not self._ffb_stop_event.is_set():
			if self.ui is None:
				time.sleep(0.01)
				continue
			try:
				ready, _, _ = select.select([self.ui.fd], [], [], 0.01)
				if ready:
					self.process_ff_events(time.monotonic())
			except (BlockingIOError, OSError):
				continue

	def tick_effects(self, now: float) -> None:
		self.effects.tick(now)


def crc_xor(payload: bytes) -> int:
	crc = 0
	for b in payload:
		crc ^= b
	return crc & 0xFF


def build_torque_packet(seq: int, torque: int, gain: int = 255, flags: int = 0) -> bytes:
	torque = max(-32768, min(32767, int(torque)))
	gain = max(0, min(255, int(gain)))
	flags = max(0, min(255, int(flags)))
	msg = bytearray(TORQUE_LEN)
	msg[0] = BRIDGE_START
	msg[1] = PKT_TORQUE
	msg[2] = seq & 0xFF
	msg[3] = torque & 0xFF
	msg[4] = (torque >> 8) & 0xFF
	msg[5] = gain
	msg[6] = flags
	msg[7] = crc_xor(msg[:-1])
	return bytes(msg)


def build_control_packet(seq: int, cmd: int, value: int = 0) -> bytes:
	cmd = max(0, min(255, int(cmd)))
	value = max(-32768, min(32767, int(value)))
	msg = bytearray(CONTROL_LEN)
	msg[0] = BRIDGE_START
	msg[1] = PKT_CONTROL
	msg[2] = seq & 0xFF
	msg[3] = cmd
	msg[4] = value & 0xFF
	msg[5] = (value >> 8) & 0xFF
	msg[6] = crc_xor(msg[:-1])
	return bytes(msg)


def send_startup_command(
	ser: serial.Serial,
	startup_flags: int,
	logger: BridgeLogger | None,
	retries: int = 3,
	retry_delay: float = 0.05,
) -> None:
	packet = build_control_packet(0, CMD_STARTUP, startup_flags)
	for attempt in range(retries):
		ser.write(packet)
		if logger and logger.path:
			logger.log(
				f"[BOOT] startup_cmd attempt={attempt + 1}/{retries} flags=0x{startup_flags:04X} pkt={packet.hex(' ')}"
			)
		time.sleep(retry_delay)


def parse_telemetry_frame(frame: bytes) -> tuple[int, int, int, int, int]:
	if len(frame) != TELEMETRY_LEN:
		raise ValueError("telemetry len invalido")
	if frame[0] != BRIDGE_START or frame[1] != PKT_TELEMETRY:
		raise ValueError("cabecera invalida")
	if crc_xor(frame[:-1]) != frame[-1]:
		raise ValueError("crc invalido")
	seq = frame[2]
	x = int.from_bytes(frame[3:5], "little", signed=True)
	y = int.from_bytes(frame[5:7], "little", signed=True)
	z = int.from_bytes(frame[7:9], "little", signed=True)
	buttons = int.from_bytes(frame[9:17], "little", signed=False)
	return seq, x, y, z, buttons


def open_serial(args: argparse.Namespace) -> serial.Serial:
	ser = serial.Serial(
		port=args.port,
		baudrate=args.baud,
		timeout=0.005,
		xonxoff=False,
		rtscts=False,
		dsrdtr=False,
	)
	try:
		ser.setDTR(False)
		ser.setRTS(False)
	except Exception:
		pass
	if args.startup_delay > 0:
		time.sleep(args.startup_delay)
	try:
		ser.reset_input_buffer()
		ser.reset_output_buffer()
	except Exception:
		pass
	return ser


def run(args: argparse.Namespace) -> int:
	logger = BridgeLogger(args.log_file)
	torque_state = TorqueState(
		max_torque=FFBEffectsManagerLinux.MAX_TORQUE,
		min_torque=FFBEffectsManagerLinux.MIN_TORQUE,
		use_min_torque_comp=FFBEffectsManagerLinux.USE_MIN_TORQUE_COMP,
		input_deadzone=FFBEffectsManagerLinux.INPUT_DEADZONE,
		min_torque_input=FFBEffectsManagerLinux.MIN_TORQUE_INPUT,
		torque_response_gamma=FFBEffectsManagerLinux.TORQUE_RESPONSE_GAMMA,
	)

	bridge = LinuxFFBBridge(
		torque_state=torque_state,
		logger=logger,
		debug_ffb=args.debug_ffb,
		debug_print_interval=args.debug_print_interval,
		max_effects=args.max_effects,
	)
	ser = open_serial(args)
	stop_event = threading.Event()

	def _sig_handler(_sig: int, _frame: object) -> None:
		stop_event.set()

	signal.signal(signal.SIGINT, _sig_handler)
	signal.signal(signal.SIGTERM, _sig_handler)

	bridge.start(
		name=args.device_name,
		vendor=args.vendor,
		product=args.product,
		version=args.version,
	)
	startup_flags = STARTUP_FLAG_AUTO_CALIB if args.auto_calib else 0
	send_startup_command(ser, startup_flags, logger)
	print(f"Bridge Linux iniciado. Serial={args.port} UInput={args.device_name}")
	print("Tip: abre jstest-gtk o evtest para verificar ejes/botones.")

	if logger.path:
		logger.log(
			f"[RUN] start serial={args.port} baud={args.baud} devname={args.device_name} vendor=0x{args.vendor:04X} product=0x{args.product:04X}"
		)

	rx = bytearray()
	tx_seq = 0
	last_print = time.monotonic()
	last_axes = (0, 0, 0)
	last_tx_debug = 0.0
	last_tx_ts = 0.0
	last_ffb_tick_ts = 0.0
	forced_torque = max(-32768, min(32767, int(args.force_torque))) if args.force_torque is not None else None
	last_buttons_state = -1
	tx_interval = 0.005
	ffb_tick_interval = 0.005

	try:
		while not stop_event.is_set():
			chunk = ser.read(256)
			if chunk:
				rx.extend(chunk)

			while len(rx) >= TELEMETRY_LEN:
				if rx[0] != BRIDGE_START:
					del rx[0]
					continue
				if len(rx) < TELEMETRY_LEN:
					break

				frame = bytes(rx[:TELEMETRY_LEN])
				try:
					seq, x, y, z, buttons = parse_telemetry_frame(frame)
					torque_state.update_motion(x)
					bridge.update_axes_buttons(x, y, z, buttons)
					if buttons != last_buttons_state:
						active_btns = [i + 1 for i in range(32) if (buttons & (1 << i))]
						msg_extras = []
						if buttons & (1 << 32):
							msg_extras.append("HAT UP")
						if buttons & (1 << 33):
							msg_extras.append("HAT LEFT")
						if buttons & (1 << 34):
							msg_extras.append("HAT RIGHT")
						if buttons & (1 << 35):
							msg_extras.append("HAT DOWN")
						print(
							f"\r[ESTADO] Botones {active_btns} | {', '.join(msg_extras)}" + " " * 20,
							flush=True,
						)
						last_buttons_state = buttons
					last_axes = (x, y, z)
					if logger.path:
						logger.log(
							f"[RX] seq={seq:3d} x={x:6d} y={y:6d} z={z:6d} buttons=0x{buttons:08X} raw={frame.hex(' ')}"
						)
				except ValueError:
					if logger.path:
						logger.log(f"[RX_ERR] invalid telemetry frame raw={frame.hex(' ')}")
					del rx[0]
					continue

				del rx[:TELEMETRY_LEN]

			now = time.monotonic()
			if (now - last_ffb_tick_ts) >= ffb_tick_interval:
				bridge.tick_effects(now)
				last_ffb_tick_ts = now

			if (now - last_tx_ts) >= tx_interval:
				torque = forced_torque if forced_torque is not None else torque_state.compute_torque()
				tx_pkt = build_torque_packet(tx_seq, torque, 255, 0)
				ser.write(tx_pkt)

				if logger.path:
					dev_gain, eff_force, eff_active, motion = torque_state.snapshot()
					logger.log(
						f"[TX] seq={tx_seq:3d} torque={torque:5d} gain={dev_gain:3d} pkt={tx_pkt.hex(' ')} effects_force={eff_force} active={eff_active} motion={motion}"
					)

				tx_seq = (tx_seq + 1) & 0xFF
				last_tx_ts = now

				if args.debug_tx and (now - last_tx_debug) >= args.debug_print_interval:
					dev_gain, eff_force, eff_active, motion = torque_state.snapshot()
					print(
						f"[TX] torque={torque:5d} gain={dev_gain:3d} pkt={tx_pkt.hex(' ')} effects_force={eff_force} active={eff_active} motion={motion}",
						flush=True,
					)
					last_tx_debug = now

			if args.verbose and now - last_print >= 0.2:
				print(
					f"axes=({last_axes[0]:6d},{last_axes[1]:6d},{last_axes[2]:6d}) torque={torque:5d}",
					flush=True,
				)
				last_print = now

			time.sleep(args.loop_sleep)

	finally:
		try:
			ser.close()
		except Exception:
			pass
		bridge.stop()
		if logger.path:
			logger.log("[RUN] stop")
		logger.close()
		print("\nBridge detenido.")

	return 0


def build_arg_parser() -> argparse.ArgumentParser:
	p = argparse.ArgumentParser(description="ESP Driving Simulator serial <-> Linux uinput FFB bridge")
	p.add_argument("--port", required=True, help="Puerto serial del ESP, ej: /dev/ttyACM0")
	p.add_argument("--baud", type=int, default=115200, help="Baudrate serial")
	p.add_argument("--loop-sleep", type=float, default=0.002, help="Sleep por iteracion")
	p.add_argument("--startup-delay", type=float, default=1.2, help="Espera al abrir serial")
	p.add_argument("--debug-ffb", action="store_true", help="Imprime eventos FFB")
	p.add_argument("--debug-tx", action="store_true", help="Imprime paquetes de torque")
	p.add_argument("--log-file", default=None, help="Ruta de log")
	p.add_argument("--debug-print-interval", type=float, default=0.05, help="Intervalo de debug")
	p.add_argument("--verbose", action="store_true", help="Imprime telemetria")
	p.add_argument("--force-torque", type=int, default=None, help="Fuerza torque fijo")
	p.add_argument("--auto-calib", action="store_true", help="Solicita auto calibracion")
	p.add_argument("--device-name", default="ESP FFB Wheel", help="Nombre del joystick virtual")
	p.add_argument("--vendor", type=lambda x: int(x, 0), default=0x1209, help="Vendor ID (hex o dec)")
	p.add_argument("--product", type=lambda x: int(x, 0), default=0xE5F0, help="Product ID (hex o dec)")
	p.add_argument("--version", type=lambda x: int(x, 0), default=0x0001, help="Version ID (hex o dec)")
	p.add_argument("--max-effects", type=int, default=96, help="Maximo de efectos FFB")
	return p


if __name__ == "__main__":
	parser = build_arg_parser()
	sys.exit(run(parser.parse_args()))
