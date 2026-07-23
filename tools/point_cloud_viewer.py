#!/usr/bin/env python3
"""M5StickC Plus / 3x SEN0628 Wi-Fi UDP点群ビューアー。"""

from __future__ import annotations

import argparse
import csv
import math
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass, replace
from tkinter import filedialog, ttk


WIFI_SSID = "M5_POINT_CLOUD"
WIFI_PASSWORD = "m5pointcloud"
UDP_PORT = 4210
MAGIC = b"PCLD"
PROTOCOL_VERSION = 5
FRAME_FORMAT_V1 = "<4sBBBBIhhhhhhh16s192HH"
FRAME_FORMAT_V2 = "<4sBBBBIhhhhhhhhhhhhhIII16s192HH"
# v3: accelZmilliGの直後にbmaAccelX/Y/ZmilliG(BMA400, TCA CH0)を追加。
FRAME_FORMAT_V3 = "<4sBBBBIhhhhhhhhhhhhhhhhIII16s192HH"
# v4: bmaAccelZmilliGの直後にtrimMilli/calRound/calPass/calDriftCdeg(キャリブ進行状況)を追加。
FRAME_FORMAT_V4 = "<4sBBBBIhhhhhhhhhhhhhhBBhhhhIII16s192HH"
# v5: calDriftCdegの直後にgyroX/Y/ZmilliDps(補正後ジャイロ)を追加。
FRAME_FORMAT_V5 = "<4sBBBBIhhhhhhhhhhhhhhBBhhhhhhhIII16s192HH"
FRAME_SIZE_V1 = struct.calcsize(FRAME_FORMAT_V1)
FRAME_SIZE_V2 = struct.calcsize(FRAME_FORMAT_V2)
FRAME_SIZE_V3 = struct.calcsize(FRAME_FORMAT_V3)
FRAME_SIZE_V4 = struct.calcsize(FRAME_FORMAT_V4)
FRAME_SIZE_V5 = struct.calcsize(FRAME_FORMAT_V5)
SENSOR_ANGLES_DEG = (0.0, 50.0, -50.0)
SENSOR_COLORS = ("#42d7ff", "#66ff88", "#ffad42")
SENSOR_NAMES = ("FRONT", "LEFT", "RIGHT")
SENSOR_FOV_DEG = 60.0
ROBOT_WIDTH_MM = 288
REQUIRED_GAP_MM = 348
DISPLAY_RANGE_MM = 1200
MAP_VIEW_RADIUS_MM = 2500
MAP_SENSOR_RANGE_MM = 2000
MAP_GRID_MM = 25
MAP_MAX_CELLS = 100_000
TRAIL_MAX_POINTS = 10_000
MAX_ODOMETRY_DT_S = 0.6
GRAVITY_MM_S2 = 9806.65
ACCEL_DEADBAND_G = 0.018
ACCEL_LOW_PASS_ALPHA = 0.35

STATE_NAMES = {0: "FORWARD", 1: "PIVOT", 2: "BACK", 3: "CALIBRATING"}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class PointCloudFrame:
    protocol_version: int
    timestamp_ms: int
    drive_state: int
    flags: int
    yaw_deg: float
    target_yaw_deg: float
    gap_target_yaw_deg: float
    left_clearance_mm: int
    right_clearance_mm: int
    motor_left: int
    motor_right: int
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    sensor_yaw_deg: tuple[float, float, float]
    sensor_timestamp_ms: tuple[int, int, int]
    reason: str
    distances: tuple[int, ...]
    sender: str = ""
    bma_accel_x_g: float = 0.0
    bma_accel_y_g: float = 0.0
    bma_accel_z_g: float = 0.0
    trim: float = 0.0
    cal_round: int = 0
    cal_pass: int = 0
    cal_drift_deg: float = 0.0
    gyro_x_dps: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0

    @property
    def drive_enabled(self) -> bool: return bool(self.flags & 0x01)

    @property
    def imu_ready(self) -> bool: return bool(self.flags & 0x02)

    @property
    def gap_target_active(self) -> bool: return bool(self.flags & 0x04)

    @property
    def am_online(self) -> bool: return bool(self.flags & 0x08)

    @property
    def wifi_client(self) -> bool: return bool(self.flags & 0x10)

    @property
    def bma_ok(self) -> bool: return bool(self.flags & 0x20)

    @property
    def builtin_imu_data_ok(self) -> bool: return bool(self.flags & 0x40)


@dataclass
class MapCell:
    x_mm: float
    y_mm: float
    hits: int
    sensor: int
    last_seen_ms: int


def norm_deg_180(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class DeadReckoningMap:
    """IMUヨー・加速度を世界座標へ変換し、速度・軌跡・点群地図を推定する。"""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_deg = 0.0
        self.velocity_x_mm_s = 0.0
        self.velocity_y_mm_s = 0.0
        self.filtered_accel_x_g = 0.0
        self.filtered_accel_y_g = 0.0
        self.distance_mm = 0.0
        self.cells: dict[tuple[int, int], MapCell] = {}
        self.trail: list[tuple[float, float]] = [(0.0, 0.0)]
        self.latest_points: list[tuple[float, float, int]] = []
        self._previous: PointCloudFrame | None = None
        self._last_sensor_timestamp: list[int | None] = [None, None, None]

    @staticmethod
    def _continuous_timestamp(previous_ms: int, current_ms: int) -> bool:
        return current_ms >= previous_ms or previous_ms - current_ms > 0x80000000

    def update(self, frame: PointCloudFrame) -> None:
        previous = self._previous
        if previous is not None:
            continuous = self._continuous_timestamp(previous.timestamp_ms, frame.timestamp_ms)
            delta_ms = (frame.timestamp_ms - previous.timestamp_ms) & 0xFFFFFFFF
            dt = delta_ms / 1000.0

            yaw_delta = 0.0
            if continuous and previous.imu_ready and frame.imu_ready:
                yaw_delta = norm_deg_180(frame.yaw_deg - previous.yaw_deg)

            if continuous and 0.0 < dt <= MAX_ODOMETRY_DT_S:
                old_vx, old_vy = self.velocity_x_mm_s, self.velocity_y_mm_s
                mid_heading = math.radians(self.heading_deg + yaw_delta * 0.5)

                # M5StickC PlusはUSB端子を車体前面、画面を車体上面にして水平取付されている。
                # 標準姿勢(画面を正面に見てUSB下)基準のIMU軸だとX+=画面右/Y+=画面上(USB反対側)/
                # Z+=画面から手前なので、この取付だとIMU X+は車体左、IMU Y+は車体後ろを向く
                # (Z+は車体上面で一致)。ロボット座標(local_x=右, local_y=前)に合わせるため、
                # 両軸とも符号反転させて読む。
                m5_accel_x_g = -(previous.accel_x_g + frame.accel_x_g) * 0.5
                m5_accel_y_g = -(previous.accel_y_g + frame.accel_y_g) * 0.5

                # BMA400(TCA CH0, v3)。コネクタが車体後ろ向きの取付から、データシートの
                # センサー座標系(Z=チップ面から手前、Y=コネクタと反対方向、X=Y×Zの右手系)と
                # 照合すると生値がそのままX=車体右/Y=車体前方になる想定(符号反転・軸入替なし)。
                # M5内蔵IMUとは別センサーなので、両方揃っている間は単純平均でブレンドし、
                # 片方しか無ければそちらだけを使う。
                bma_available = previous.bma_ok and frame.bma_ok
                if bma_available:
                    bma_x_g = (previous.bma_accel_x_g + frame.bma_accel_x_g) * 0.5
                    bma_y_g = (previous.bma_accel_y_g + frame.bma_accel_y_g) * 0.5
                    accel_x_g = (m5_accel_x_g + bma_x_g) * 0.5
                    accel_y_g = (m5_accel_y_g + bma_y_g) * 0.5
                else:
                    accel_x_g = m5_accel_x_g
                    accel_y_g = m5_accel_y_g
                if abs(accel_x_g) < ACCEL_DEADBAND_G:
                    accel_x_g = 0.0
                if abs(accel_y_g) < ACCEL_DEADBAND_G:
                    accel_y_g = 0.0
                self.filtered_accel_x_g += ACCEL_LOW_PASS_ALPHA * (accel_x_g - self.filtered_accel_x_g)
                self.filtered_accel_y_g += ACCEL_LOW_PASS_ALPHA * (accel_y_g - self.filtered_accel_y_g)

                pivoting = previous.drive_state == 1 or previous.motor_left * previous.motor_right < 0
                if not previous.drive_enabled:
                    self.velocity_x_mm_s = 0.0
                    self.velocity_y_mm_s = 0.0
                elif pivoting:
                    # 超信地旋回中は並進させず、向きだけIMUヨーで更新する。
                    self.velocity_x_mm_s = 0.0
                    self.velocity_y_mm_s = 0.0
                else:
                    # モーター指令からの推定最高速度モデルは使わず、M5内蔵IMUとBMA400
                    # (2基分)の実測加速度を積分した値のみで速度・移動量を推定する。
                    body_ax = self.filtered_accel_x_g * GRAVITY_MM_S2
                    body_ay = self.filtered_accel_y_g * GRAVITY_MM_S2
                    world_ax = body_ax * math.cos(mid_heading) - body_ay * math.sin(mid_heading)
                    world_ay = body_ax * math.sin(mid_heading) + body_ay * math.cos(mid_heading)
                    self.velocity_x_mm_s += world_ax * dt
                    self.velocity_y_mm_s += world_ay * dt

                if not previous.drive_enabled or pivoting:
                    dx = dy = 0.0
                else:
                    dx = (old_vx + self.velocity_x_mm_s) * 0.5 * dt
                    dy = (old_vy + self.velocity_y_mm_s) * 0.5 * dt
                self.x_mm += dx
                self.y_mm += dy
                travel = math.hypot(dx, dy)
                self.distance_mm += travel
                if travel >= 2.0:
                    self.trail.append((self.x_mm, self.y_mm))
                    if len(self.trail) > TRAIL_MAX_POINTS:
                        del self.trail[:len(self.trail) - TRAIL_MAX_POINTS]

            elif not continuous:
                self.velocity_x_mm_s = 0.0
                self.velocity_y_mm_s = 0.0

            self.heading_deg = norm_deg_180(self.heading_deg + yaw_delta)

        self._previous = frame
        self.latest_points = self._frame_points(frame)
        new_sensors = [
            sensor for sensor in range(3)
            if frame.sensor_timestamp_ms[sensor] != self._last_sensor_timestamp[sensor]
        ]
        for sensor in new_sensors:
            self._last_sensor_timestamp[sensor] = frame.sensor_timestamp_ms[sensor]
        for x_mm, y_mm, sensor in self._frame_points(frame, new_sensors):
            key = (round(x_mm / MAP_GRID_MM), round(y_mm / MAP_GRID_MM))
            cell = self.cells.get(key)
            if cell is None:
                if len(self.cells) >= MAP_MAX_CELLS:
                    continue
                self.cells[key] = MapCell(x_mm, y_mm, 1, sensor, frame.timestamp_ms)
            else:
                weight = min(cell.hits, 31)
                cell.x_mm = (cell.x_mm * weight + x_mm) / (weight + 1)
                cell.y_mm = (cell.y_mm * weight + y_mm) / (weight + 1)
                cell.hits += 1
                cell.sensor = sensor
                cell.last_seen_ms = frame.timestamp_ms

    @property
    def speed_mm_s(self) -> float:
        return math.hypot(self.velocity_x_mm_s, self.velocity_y_mm_s)

    def _frame_points(self, frame: PointCloudFrame,
                      sensors: list[int] | range = range(3)) -> list[tuple[float, float, int]]:
        points: list[tuple[float, float, int]] = []
        pixel_step = SENSOR_FOV_DEG / 8.0
        first = SENSOR_FOV_DEG / 2.0 - pixel_step / 2.0

        # 縦8画素は同じ水平角を観測するため、各列の中央値を地図へ1点だけ登録する。
        for sensor in sensors:
            # 旋回中は測距からUDP送信までにヨーが変わるため、センサーごとの
            # 測距時ヨーを現在の世界方位へ戻してから点群を固定する。
            capture_heading_deg = self.heading_deg + norm_deg_180(
                frame.sensor_yaw_deg[sensor] - frame.yaw_deg
            )
            heading = math.radians(capture_heading_deg)
            for col in range(8):
                samples = [
                    frame.distances[sensor * 64 + row * 8 + col]
                    for row in range(8)
                    if 20 <= frame.distances[sensor * 64 + row * 8 + col] <= MAP_SENSOR_RANGE_MM
                ]
                if not samples:
                    continue
                samples.sort()
                distance = samples[len(samples) // 2]
                ray = math.radians(SENSOR_ANGLES_DEG[sensor] + first - col * pixel_step)
                local_x = -distance * math.sin(ray)
                local_y = distance * math.cos(ray)
                world_x = self.x_mm + local_x * math.cos(heading) - local_y * math.sin(heading)
                world_y = self.y_mm + local_x * math.sin(heading) + local_y * math.cos(heading)
                points.append((world_x, world_y, sensor))
        return points


def decode_frame(raw: bytes, sender: str = "") -> PointCloudFrame:
    if len(raw) not in (FRAME_SIZE_V1, FRAME_SIZE_V2, FRAME_SIZE_V3, FRAME_SIZE_V4, FRAME_SIZE_V5) or raw[:4] != MAGIC:
        raise ValueError("invalid frame")
    crc_offset = len(raw) - 2
    expected_crc = struct.unpack_from("<H", raw, crc_offset)[0]
    if crc16_ccitt(raw[:crc_offset]) != expected_crc:
        raise ValueError("CRC mismatch")
    version = raw[4]
    bma_accel_x_g = bma_accel_y_g = bma_accel_z_g = 0.0
    trim = 0.0
    cal_round = cal_pass = 0
    cal_drift_deg = 0.0
    gyro_x_dps = gyro_y_dps = gyro_z_dps = 0.0
    if version == 1 and len(raw) == FRAME_SIZE_V1:
        values = struct.unpack(FRAME_FORMAT_V1, raw)
        accel_x_g = accel_y_g = accel_z_g = 0.0
        sensor_yaw_deg = (values[6] / 100.0,) * 3
        sensor_timestamp_ms = (values[5],) * 3
        reason_index, distance_index = 13, 14
    elif version == 2 and len(raw) == FRAME_SIZE_V2:
        values = struct.unpack(FRAME_FORMAT_V2, raw)
        accel_x_g, accel_y_g, accel_z_g = values[13] / 1000.0, values[14] / 1000.0, values[15] / 1000.0
        sensor_yaw_deg = (values[16] / 100.0, values[17] / 100.0, values[18] / 100.0)
        sensor_timestamp_ms = (values[19], values[20], values[21])
        reason_index, distance_index = 22, 23
    elif version == 3 and len(raw) == FRAME_SIZE_V3:
        values = struct.unpack(FRAME_FORMAT_V3, raw)
        accel_x_g, accel_y_g, accel_z_g = values[13] / 1000.0, values[14] / 1000.0, values[15] / 1000.0
        # v3: BMA400(TCA CH0)。コネクタ後ろ向き取付+データシート軸図から、
        # 生値のX=車体右/Y=車体前方に既に一致すると判断(軸補正なしでそのまま使用)。
        bma_accel_x_g, bma_accel_y_g, bma_accel_z_g = values[16] / 1000.0, values[17] / 1000.0, values[18] / 1000.0
        sensor_yaw_deg = (values[19] / 100.0, values[20] / 100.0, values[21] / 100.0)
        sensor_timestamp_ms = (values[22], values[23], values[24])
        reason_index, distance_index = 25, 26
    elif version == 4 and len(raw) == FRAME_SIZE_V4:
        values = struct.unpack(FRAME_FORMAT_V4, raw)
        accel_x_g, accel_y_g, accel_z_g = values[13] / 1000.0, values[14] / 1000.0, values[15] / 1000.0
        bma_accel_x_g, bma_accel_y_g, bma_accel_z_g = values[16] / 1000.0, values[17] / 1000.0, values[18] / 1000.0
        # v4: キャリブレーション(BtnB)の進行状況。非キャリブ中はcal_round/cal_pass=0、
        # cal_drift_degは直近ラウンド確定時の値が残る。
        trim = values[19] / 1000.0
        cal_round, cal_pass = values[20], values[21]
        cal_drift_deg = values[22] / 100.0
        sensor_yaw_deg = (values[23] / 100.0, values[24] / 100.0, values[25] / 100.0)
        sensor_timestamp_ms = (values[26], values[27], values[28])
        reason_index, distance_index = 29, 30
    elif version == PROTOCOL_VERSION and len(raw) == FRAME_SIZE_V5:
        values = struct.unpack(FRAME_FORMAT_V5, raw)
        accel_x_g, accel_y_g, accel_z_g = values[13] / 1000.0, values[14] / 1000.0, values[15] / 1000.0
        bma_accel_x_g, bma_accel_y_g, bma_accel_z_g = values[16] / 1000.0, values[17] / 1000.0, values[18] / 1000.0
        trim = values[19] / 1000.0
        cal_round, cal_pass = values[20], values[21]
        cal_drift_deg = values[22] / 100.0
        gyro_x_dps, gyro_y_dps, gyro_z_dps = values[23] / 1000.0, values[24] / 1000.0, values[25] / 1000.0
        sensor_yaw_deg = (values[26] / 100.0, values[27] / 100.0, values[28] / 100.0)
        sensor_timestamp_ms = (values[29], values[30], values[31])
        reason_index, distance_index = 32, 33
    else:
        raise ValueError(f"unsupported protocol version/size {version}/{len(raw)}")
    return PointCloudFrame(
        protocol_version=version,
        timestamp_ms=values[5], drive_state=values[2], flags=values[3],
        yaw_deg=values[6] / 100.0, target_yaw_deg=values[7] / 100.0,
        gap_target_yaw_deg=values[8] / 100.0,
        left_clearance_mm=values[9], right_clearance_mm=values[10],
        motor_left=values[11], motor_right=values[12],
        accel_x_g=accel_x_g, accel_y_g=accel_y_g, accel_z_g=accel_z_g,
        sensor_yaw_deg=sensor_yaw_deg,
        sensor_timestamp_ms=sensor_timestamp_ms,
        reason=values[reason_index].split(b"\0", 1)[0].decode("ascii", errors="replace"),
        distances=tuple(values[distance_index:distance_index + 192]), sender=sender,
        bma_accel_x_g=bma_accel_x_g, bma_accel_y_g=bma_accel_y_g, bma_accel_z_g=bma_accel_z_g,
        trim=trim, cal_round=cal_round, cal_pass=cal_pass, cal_drift_deg=cal_drift_deg,
        gyro_x_dps=gyro_x_dps, gyro_y_dps=gyro_y_dps, gyro_z_dps=gyro_z_dps,
    )


class UdpFrameReceiver:
    def __init__(self, output_queue: queue.Queue, port: int):
        self._queue = output_queue
        self._port = port
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._socket:
            self._socket.close()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _publish(self, kind: str, payload) -> None:
        try:
            self._queue.put_nowait((kind, payload))
        except queue.Full:
            try: self._queue.get_nowait()
            except queue.Empty: pass
            self._queue.put_nowait((kind, payload))

    def _run(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket = sock
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind(("0.0.0.0", self._port))
            sock.settimeout(0.5)
            self._publish("status", f"UDP :{self._port} 受信待機中")
            while not self._stop.is_set():
                try:
                    raw, address = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                try:
                    frame = decode_frame(raw, address[0])
                except ValueError:
                    continue
                self._publish("frame", frame)
        except OSError as exc:
            if not self._stop.is_set(): self._publish("status", f"UDPエラー: {exc}")


class PointCloudViewer(tk.Tk):
    def __init__(self, port: int = UDP_PORT):
        super().__init__()
        self.title("M5 SEN0628 Wi-Fi Point Cloud")
        self.geometry("1180x780")
        self.minsize(900, 620)
        self._messages: queue.Queue = queue.Queue(maxsize=8)
        self._receiver = UdpFrameReceiver(self._messages, port)
        self._frame: PointCloudFrame | None = None
        self._last_frame_at = 0.0
        self._count = 0
        self._fps_at = time.monotonic()
        self._map = DeadReckoningMap()
        self._view_x_mm = 0.0
        self._view_y_mm = 0.0
        self._follow_robot = True
        self._zoom = 1.0
        self._last_scale = 1.0
        self._drag_last: tuple[int, int] | None = None
        self._follow_var = tk.StringVar(value="追従: ON")

        self.status_var = tk.StringVar(value=f"Wi-Fi「{WIFI_SSID}」へ接続してください")
        self.fps_var = tk.StringVar(value="0.0 fps")
        self.state_var = tk.StringVar(value="-")
        self.reason_var = tk.StringVar(value="-")
        self.pose_var = tk.StringVar(value="-")
        self.clear_var = tk.StringVar(value="-")
        self.motor_var = tk.StringVar(value="-")
        self.link_var = tk.StringVar(value="-")
        self.map_var = tk.StringVar(value="-")
        self.cal_var = tk.StringVar(value="-")
        self._build_ui(port)
        self._receiver.start()
        self.after(30, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build_ui(self, port: int) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT)
        ttk.Label(top, text=f"SSID: {WIFI_SSID}  PASS: {WIFI_PASSWORD}  UDP:{port}").pack(side=tk.LEFT, padx=20)
        ttk.Label(top, textvariable=self.fps_var).pack(side=tk.RIGHT)

        body = ttk.Frame(self, padding=(8, 0, 8, 8))
        body.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(body, bg="#071018", highlightthickness=0, cursor="fleur")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._draw())
        self.canvas.bind("<ButtonPress-1>", self._on_pan_start)
        self.canvas.bind("<B1-Motion>", self._on_pan_move)
        self.canvas.bind("<ButtonRelease-1>", self._on_pan_end)
        self.canvas.bind("<MouseWheel>", self._on_zoom)
        self.canvas.bind("<Button-4>", self._on_zoom)
        self.canvas.bind("<Button-5>", self._on_zoom)
        side = ttk.Frame(body, width=260, padding=(12, 4))
        side.pack(side=tk.RIGHT, fill=tk.Y)
        side.pack_propagate(False)
        for title, var in (("STATE", self.state_var), ("REASON", self.reason_var),
                           ("IMU / TARGET", self.pose_var), ("CLEARANCE", self.clear_var),
                           ("MOTOR", self.motor_var), ("CALIBRATION", self.cal_var),
                           ("MAP", self.map_var), ("LINK", self.link_var)):
            ttk.Label(side, text=title, font=("TkDefaultFont", 9, "bold")).pack(anchor=tk.W, pady=(8, 0))
            ttk.Label(side, textvariable=var, wraplength=230).pack(anchor=tk.W)
        ttk.Separator(side).pack(fill=tk.X, pady=12)
        ttk.Button(side, textvariable=self._follow_var, command=self._toggle_follow).pack(fill=tk.X)
        ttk.Button(side, text="マップをCSV保存", command=self._save_map).pack(fill=tk.X, pady=(6, 0))
        ttk.Button(side, text="マップを消去", command=self._reset_map).pack(fill=tk.X, pady=(6, 0))
        ttk.Label(side, text="ドラッグ: 地図を移動 / ホイール: 拡大縮小").pack(anchor=tk.W, pady=(8, 0))
        for name, color in zip(SENSOR_NAMES, SENSOR_COLORS):
            tk.Label(side, text=f"● {name}", fg=color, bg=self.cget("bg")).pack(anchor=tk.W)
        ttk.Label(
            side,
            text=("速度推定: IMU2基(M5内蔵+BMA400)の加速度積分(実測)\n"
                  "白破線: 必要幅348mm\n黄線: 目標方位\n白線: 推定走行軌跡"),
        ).pack(anchor=tk.W, pady=10)

    def _poll(self) -> None:
        try:
            while True:
                kind, payload = self._messages.get_nowait()
                if kind == "status": self.status_var.set(payload)
                elif kind == "frame":
                    self._frame = payload
                    self._map.update(payload)
                    self._last_frame_at = time.monotonic()
                    self._count += 1
                    self.status_var.set(f"受信中: {payload.sender}:{UDP_PORT}")
                    self._update_labels(payload)
                    self._draw()
        except queue.Empty:
            pass
        now = time.monotonic()
        if now - self._fps_at >= 1.0:
            self.fps_var.set(f"{self._count / (now - self._fps_at):.1f} fps")
            self._count, self._fps_at = 0, now
            if self._last_frame_at and now - self._last_frame_at > 2.0:
                self.status_var.set("受信停止: M5電源とWi-Fi接続を確認")
        self.after(30, self._poll)

    def _update_labels(self, f: PointCloudFrame) -> None:
        self.state_var.set(STATE_NAMES.get(f.drive_state, f"UNKNOWN({f.drive_state})"))
        self.reason_var.set(f.reason or "-")
        bma_line = (
            f"BMA X {f.bma_accel_x_g:+.3f}g / Y {f.bma_accel_y_g:+.3f}g"
            if f.bma_ok else "BMA ---"
        )
        gyro_line = (
            f"Gyro X {f.gyro_x_dps:+.2f} / Y {f.gyro_y_dps:+.2f} / Z {f.gyro_z_dps:+.2f} °/s"
            if f.protocol_version >= 5 else
            f"Gyro --- (M5 firmware v{f.protocol_version}: v5 required)"
        )
        self.pose_var.set(
            f"Yaw {f.yaw_deg:+.1f}° / Target {f.target_yaw_deg:+.1f}°\n"
            f"{gyro_line}\n"
            f"Map heading {self._map.heading_deg:+.1f}°\n"
            f"M5  X {f.accel_x_g:+.3f}g / Y {f.accel_y_g:+.3f}g\n"
            f"{bma_line}"
        )
        self.clear_var.set(f"Left {f.left_clearance_mm}mm\nRight {f.right_clearance_mm}mm")
        self.motor_var.set(f"L {f.motor_left:+d} / R {f.motor_right:+d}")
        self.map_var.set(
            f"X {self._map.x_mm:+.0f}mm / Y {self._map.y_mm:+.0f}mm\n"
            f"速度 {self._map.speed_mm_s:.0f}mm/s / 走行 {self._map.distance_mm:.0f}mm\n"
            f"セル {len(self._map.cells)}"
        )
        self.link_var.set(
            f"Protocol v{f.protocol_version}\n"
            f"Drive {'ON' if f.drive_enabled else 'OFF'}\nIMU {'OK' if f.imu_ready else 'WAIT'}\n"
            f"M5 sensor {'DATA' if f.builtin_imu_data_ok else 'NO DATA'}\n"
            f"AM {'ONLINE' if f.am_online else 'OFFLINE'}\n"
            f"Wi-Fi {'CLIENT' if f.wifi_client else 'NO CLIENT'}\n"
            f"BMA400 {'OK' if f.bma_ok else 'NG/WAIT'}"
        )
        if f.drive_state == 3:  # DS_CALIBRATING
            self.cal_var.set(
                f"Round {f.cal_round} / Pass {f.cal_pass}\n"
                f"Drift(直近ラウンド) {f.cal_drift_deg:+.1f}°\n"
                f"Trim {f.trim:+.3f}\n{f.reason}"
            )
        elif f.cal_round or f.cal_pass:
            self.cal_var.set(f"(前回) Drift {f.cal_drift_deg:+.1f}° / Trim {f.trim:+.3f}")
        else:
            self.cal_var.set(f"Trim {f.trim:+.3f}")

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 100), max(c.winfo_height(), 100)
        ox, oy = w / 2, h / 2
        if self._follow_robot:
            self._view_x_mm = self._map.x_mm
            self._view_y_mm = self._map.y_mm
        scale = min((w - 60) / (MAP_VIEW_RADIUS_MM * 2), (h - 60) / (MAP_VIEW_RADIUS_MM * 2)) * self._zoom
        self._last_scale = scale
        self._draw_world_grid(ox, oy, scale, w, h)
        if not self._frame:
            c.create_text(ox, h / 2, text="Wi-Fi点群を待っています", fill="#8ca0ad", font=("TkDefaultFont", 16))
            return

        f = self._frame
        for cell in self._map.cells.values():
            x, y = self._world_to_screen(cell.x_mm, cell.y_mm, ox, oy, scale)
            if -3 <= x <= w + 3 and -3 <= y <= h + 3:
                radius = 1 if cell.hits < 3 else 2
                c.create_oval(x-radius, y-radius, x+radius, y+radius,
                              fill=SENSOR_COLORS[cell.sensor], outline="")

        if len(self._map.trail) >= 2:
            coords: list[float] = []
            for x_mm, y_mm in self._map.trail:
                x, y = self._world_to_screen(x_mm, y_mm, ox, oy, scale)
                coords.extend((x, y))
            c.create_line(*coords, fill="#e8edf2", width=2)

        for x_mm, y_mm, sensor in self._map.latest_points:
            x, y = self._world_to_screen(x_mm, y_mm, ox, oy, scale)
            if -4 <= x <= w + 4 and -4 <= y <= h + 4:
                c.create_oval(x-3, y-3, x+3, y+3, fill=SENSOR_COLORS[sensor], outline="#ffffff")

        target_heading = self._map.heading_deg + norm_deg_180(f.target_yaw_deg - f.yaw_deg)
        self._draw_corridor(ox, oy, scale, math.radians(target_heading))
        self._draw_robot(ox, oy, scale, math.radians(self._map.heading_deg))

    def _world_to_screen(self, x_mm: float, y_mm: float, ox: float, oy: float,
                         scale: float) -> tuple[float, float]:
        return (
            ox + (x_mm - self._view_x_mm) * scale,
            oy - (y_mm - self._view_y_mm) * scale,
        )

    def _draw_world_grid(self, ox: float, oy: float, scale: float, w: int, h: int) -> None:
        spacing = 500
        min_x = self._view_x_mm - w / (2 * scale)
        max_x = self._view_x_mm + w / (2 * scale)
        min_y = self._view_y_mm - h / (2 * scale)
        max_y = self._view_y_mm + h / (2 * scale)
        start_x = math.floor(min_x / spacing) * spacing
        start_y = math.floor(min_y / spacing) * spacing

        x_mm = start_x
        while x_mm <= max_x:
            x, _ = self._world_to_screen(x_mm, self._view_y_mm, ox, oy, scale)
            self.canvas.create_line(x, 0, x, h, fill="#20303a")
            self.canvas.create_text(x + 3, h - 4, text=f"{x_mm / 1000:.1f}m", fill="#627785", anchor=tk.SW)
            x_mm += spacing

        y_mm = start_y
        while y_mm <= max_y:
            _, y = self._world_to_screen(self._view_x_mm, y_mm, ox, oy, scale)
            self.canvas.create_line(0, y, w, y, fill="#20303a")
            self.canvas.create_text(4, y - 3, text=f"{y_mm / 1000:.1f}m", fill="#627785", anchor=tk.SW)
            y_mm += spacing

    @staticmethod
    def _rotate_local(x_mm: float, y_mm: float, heading: float) -> tuple[float, float]:
        return (
            x_mm * math.cos(heading) - y_mm * math.sin(heading),
            x_mm * math.sin(heading) + y_mm * math.cos(heading),
        )

    def _draw_robot(self, ox: float, oy: float, scale: float, heading: float) -> None:
        half_width = ROBOT_WIDTH_MM / 2
        half_length = 130.0
        polygon: list[float] = []
        for local_x, local_y in ((-half_width, -half_length), (half_width, -half_length),
                                 (half_width, half_length), (-half_width, half_length)):
            dx, dy = self._rotate_local(local_x, local_y, heading)
            polygon.extend((ox + dx * scale, oy - dy * scale))
        self.canvas.create_polygon(*polygon, fill="#3454a5", outline="#9eb8ff", width=2)
        fx, fy = self._rotate_local(0.0, half_length + 90.0, heading)
        self.canvas.create_line(ox, oy, ox + fx * scale, oy - fy * scale,
                                fill="#dce7ff", width=3, arrow=tk.LAST)

    def _draw_corridor(self, ox: float, oy: float, scale: float, heading: float) -> None:
        forward_x, forward_y = -math.sin(heading), math.cos(heading)
        side_x, side_y = math.cos(heading), math.sin(heading)
        length, half = DISPLAY_RANGE_MM * scale, REQUIRED_GAP_MM * scale / 2
        for sign in (-1, 1):
            x = ox + side_x * half * sign
            y = oy - side_y * half * sign
            self.canvas.create_line(x, y, x + forward_x * length, y - forward_y * length,
                                    fill="#d9e1e6", dash=(6, 5))
        self.canvas.create_line(ox, oy, ox + forward_x * length * .75, oy - forward_y * length * .75,
                                fill="#ffe05c", width=2, arrow=tk.LAST)

    def _on_pan_start(self, event: tk.Event) -> None:
        self._follow_robot = False
        self._follow_var.set("追従: OFF")
        self._drag_last = (event.x, event.y)

    def _on_pan_move(self, event: tk.Event) -> None:
        if self._drag_last is None:
            return
        dx = event.x - self._drag_last[0]
        dy = event.y - self._drag_last[1]
        self._drag_last = (event.x, event.y)
        self._view_x_mm -= dx / self._last_scale
        self._view_y_mm += dy / self._last_scale
        self._draw()

    def _on_pan_end(self, _event: tk.Event) -> None:
        self._drag_last = None

    def _on_zoom(self, event: tk.Event) -> None:
        zooming_in = getattr(event, "delta", 0) > 0 or getattr(event, "num", None) == 4
        factor = 1.1 if zooming_in else 1 / 1.1
        self._zoom = min(8.0, max(0.2, self._zoom * factor))
        self._draw()

    def _toggle_follow(self) -> None:
        self._follow_robot = not self._follow_robot
        self._follow_var.set("追従: ON" if self._follow_robot else "追従: OFF")
        self._draw()

    def _reset_map(self) -> None:
        self._map.reset()
        self._view_x_mm = 0.0
        self._view_y_mm = 0.0
        self._zoom = 1.0
        self._follow_robot = True
        self._follow_var.set("追従: ON")
        self.map_var.set("X +0mm / Y +0mm\n速度 0mm/s / 走行 0mm\nセル 0")
        self._draw()

    def _save_map(self) -> None:
        path = filedialog.asksaveasfilename(
            parent=self,
            title="点群マップをCSV保存",
            defaultextension=".csv",
            filetypes=(("CSV", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("x_mm", "y_mm", "hits", "sensor", "last_seen_ms"))
            for cell in self._map.cells.values():
                writer.writerow((f"{cell.x_mm:.1f}", f"{cell.y_mm:.1f}", cell.hits,
                                 SENSOR_NAMES[cell.sensor], cell.last_seen_ms))
        self.status_var.set(f"CSV保存: {path}")

    def _close(self) -> None:
        self._receiver.close()
        self.destroy()


def run_self_test() -> None:
    distances = [0xFFFF] * 192
    distances[27] = 500
    values = [MAGIC, 2, 0, 0x1F, 0, 123456, 125, 250, 250, 80, 90, 190, 188,
              25, -40, 5, 100, 110, 120,
              123400, 123410, 123420,
              b"SELF_TEST".ljust(16, b"\0"), *distances, 0]
    raw = bytearray(struct.pack(FRAME_FORMAT_V2, *values))
    struct.pack_into("<H", raw, len(raw) - 2, crc16_ccitt(raw[:-2]))
    frame = decode_frame(bytes(raw), "127.0.0.1")
    assert FRAME_SIZE_V2 == 452 and frame.distances[27] == 500 and frame.reason == "SELF_TEST"
    assert frame.accel_x_g == 0.025 and frame.accel_y_g == -0.040 and frame.accel_z_g == 0.005
    assert frame.sensor_yaw_deg == (1.0, 1.1, 1.2)
    assert frame.sensor_timestamp_ms == (123400, 123410, 123420)

    legacy_values = [MAGIC, 1, 0, 0x03, 0, 123456, 125, 250, 250, 80, 90, 190, 188,
                     b"V1_TEST".ljust(16, b"\0"), *distances, 0]
    legacy_raw = bytearray(struct.pack(FRAME_FORMAT_V1, *legacy_values))
    struct.pack_into("<H", legacy_raw, len(legacy_raw) - 2, crc16_ccitt(legacy_raw[:-2]))
    legacy = decode_frame(bytes(legacy_raw), "127.0.0.1")
    assert legacy.reason == "V1_TEST" and legacy.accel_x_g == 0.0

    # v3: BMA400(TCA CH0)追加分。flags=0x3Fでbma_ok(bit5)も立てる。
    v3_values = [MAGIC, 3, 0, 0x3F, 0, 123456, 125, 250, 250, 80, 90, 190, 188,
                 25, -40, 5,
                 10, -20, 1000,
                 100, 110, 120,
                 123400, 123410, 123420,
                 b"SELF_TEST_V3".ljust(16, b"\0"), *distances, 0]
    v3_raw = bytearray(struct.pack(FRAME_FORMAT_V3, *v3_values))
    struct.pack_into("<H", v3_raw, len(v3_raw) - 2, crc16_ccitt(v3_raw[:-2]))
    frame_v3 = decode_frame(bytes(v3_raw), "127.0.0.1")
    assert FRAME_SIZE_V3 == 458 and frame_v3.bma_ok
    assert frame_v3.bma_accel_x_g == 0.010 and frame_v3.bma_accel_y_g == -0.020 and frame_v3.bma_accel_z_g == 1.000
    assert frame_v3.sensor_yaw_deg == (1.0, 1.1, 1.2)

    # v4: キャリブ進行状況(trimMilli/calRound/calPass/calDriftCdeg)追加分。
    v4_values = [MAGIC, 4, 3, 0x3F, 0, 123456, 125, 250, 250, 80, 90, 190, 188,
                 25, -40, 5,
                 10, -20, 1000,
                 150,
                 2, 3,
                 -320,
                 100, 110, 120,
                 123400, 123410, 123420,
                 b"CAL R2 P3/3".ljust(16, b"\0"), *distances, 0]
    v4_raw = bytearray(struct.pack(FRAME_FORMAT_V4, *v4_values))
    struct.pack_into("<H", v4_raw, len(v4_raw) - 2, crc16_ccitt(v4_raw[:-2]))
    frame_v4 = decode_frame(bytes(v4_raw), "127.0.0.1")
    assert FRAME_SIZE_V4 == 464 and frame_v4.drive_state == 3
    assert frame_v4.trim == 0.150 and frame_v4.cal_round == 2 and frame_v4.cal_pass == 3
    assert abs(frame_v4.cal_drift_deg - (-3.20)) < 1e-9
    assert frame_v4.reason == "CAL R2 P3/3"

    # v5: 補正後の内蔵IMUジャイロXYZを追加。
    v5_values = [MAGIC, 5, 3, 0x3F, 0, 123456, 125, 250, 250, 80, 90, 190, 188,
                 25, -40, 5,
                 10, -20, 1000,
                 71,
                 2, 3,
                 -710,
                 1250, -2500, -710,
                 100, 110, 120,
                 123400, 123410, 123420,
                 b"CAL R2 P3/3".ljust(16, b"\0"), *distances, 0]
    v5_raw = bytearray(struct.pack(FRAME_FORMAT_V5, *v5_values))
    struct.pack_into("<H", v5_raw, len(v5_raw) - 2, crc16_ccitt(v5_raw[:-2]))
    frame_v5 = decode_frame(bytes(v5_raw), "127.0.0.1")
    assert FRAME_SIZE_V5 == 470 and frame_v5.drive_state == 3
    assert frame_v5.trim == 0.071 and frame_v5.cal_drift_deg == -7.10
    assert (frame_v5.gyro_x_dps, frame_v5.gyro_y_dps, frame_v5.gyro_z_dps) == (1.25, -2.5, -0.71)
    assert frame_v5.sensor_yaw_deg == (1.0, 1.1, 1.2)

    # BMA400ブレンド: bma_ok時は平均、bma不可時はM5のみになることを確認。
    # accel_x_gはM5の生値(update()内でworld_x向けに符号反転される)なので、
    # world側で+0.2gにしたい場合は-0.2gを入れておく。
    blend_base = replace(frame_v3, timestamp_ms=1000, drive_state=0, flags=0x23,
                          yaw_deg=0.0, motor_left=0, motor_right=0,
                          accel_x_g=-0.2, accel_y_g=0.0, accel_z_g=0.0,
                          bma_accel_x_g=0.0, bma_accel_y_g=0.0, bma_accel_z_g=0.0)
    blend_mapper = DeadReckoningMap()
    blend_mapper.update(blend_base)
    blend_mapper.update(replace(blend_base, timestamp_ms=1100))
    blended_vx = blend_mapper.velocity_x_mm_s

    solo_mapper = DeadReckoningMap()
    solo_base = replace(blend_base, flags=0x03)  # bit5を落としbma_ok=False
    solo_mapper.update(solo_base)
    solo_mapper.update(replace(solo_base, timestamp_ms=1100))
    solo_vx = solo_mapper.velocity_x_mm_s

    assert solo_vx > 0.0
    assert 0.0 < blended_vx < solo_vx

    # 速度・移動量はモーター指令ではなく実測加速度(IMU2基)積分のみで生成されることを確認。
    mapper = DeadReckoningMap()
    moving = replace(frame, timestamp_ms=1000, flags=0x03, yaw_deg=0.0,
                     motor_left=255, motor_right=255,
                     accel_x_g=0.0, accel_y_g=-0.1, accel_z_g=0.0)
    mapper.update(moving)
    mapped_hits = sum(cell.hits for cell in mapper.cells.values())
    mapper.update(replace(moving, timestamp_ms=1200))
    assert abs(mapper.x_mm) < 0.01 and mapper.y_mm > 0.0
    assert mapper.speed_mm_s > 0.0 and mapper.distance_mm > 0.0 and mapper.cells
    assert sum(cell.hits for cell in mapper.cells.values()) == mapped_hits

    pivoting = replace(moving, timestamp_ms=1400, yaw_deg=20.0,
                       motor_left=205, motor_right=-205)
    mapper.update(pivoting)
    before_x, before_y = mapper.x_mm, mapper.y_mm
    mapper.update(replace(pivoting, timestamp_ms=1600, yaw_deg=40.0))
    assert abs(mapper.x_mm - before_x) < 0.01 and abs(mapper.y_mm - before_y) < 0.01
    assert mapper.speed_mm_s == 0.0
    assert abs(mapper.heading_deg - 40.0) < 0.01
    print(f"self-test OK: v1={FRAME_SIZE_V1}, v2={FRAME_SIZE_V2}, v3={FRAME_SIZE_V3}, "
          f"v4={FRAME_SIZE_V4}, v5={FRAME_SIZE_V5}, udp_port={UDP_PORT}, accel=OK, "
          "gyro=OK, turn_tracking=OK, mapping=OK, bma_blend=OK, cal_stream=OK")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP待受ポート")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test: run_self_test()
    else: PointCloudViewer(args.port).mainloop()


if __name__ == "__main__":
    main()
