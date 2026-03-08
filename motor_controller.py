# -*- coding: utf-8 -*-
import logging
import time

import can
import serial.tools.list_ports

from config.motor_cmd import AngleCommand, AccelCommand, PidCommand

log = logging.getLogger(__name__)

# CAN adapter VID/PID (CANable / slcan)
_ADAPTER_VID = 0x16D0
_ADAPTER_PID = 0x117E


class MotorController:
    """CAN bus motor controller.

    Usage:
        mc = MotorController()
        if not mc.connect():
            raise ConnectionError("Failed to connect to CAN bus.")

        try:
            mc.move_motor_to_angle(AngleCommand("left_joint1", 90, 360))
        finally:
            mc.disconnect()

    Or:
        with MotorController() as mc:
            mc.move_motor_to_angle(AngleCommand("left_joint1", 90, 360))
    """

    def __init__(self, bitrate: int = 1000000):
        self.bitrate = bitrate
        self.bus: can.BusABC | None = None
        self.port: str | None = None
        self._is_connected = False
        self.port = "COM5"

    def connect(self) -> bool:
        try:
            if self.port is not None:
                try:
                    self.bus = can.interface.Bus(
                        interface="slcan",
                        channel=self.port,
                        bitrate=self.bitrate,
                    )
                    log.info("CAN bus initialized on %s", self.port)

                except Exception:
                    log.warning("Real CAN not found. Using virtual CAN.")
                    self.bus = can.interface.Bus(interface="virtual")

            else:
                self.bus = can.interface.Bus(interface="virtual")
                log.info("Virtual CAN bus started")

            self._is_connected = True
            return True

        except Exception as e:
            log.error("CAN bus init failed: %s", e)
            return False

    def disconnect(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
                log.info("CAN bus disconnected.")
            except Exception as e:
                log.warning("Error during CAN shutdown: %s", e)
            finally:
                self.bus = None
                self.port = None
                self._is_connected = False

    def __enter__(self):
        if not self.connect():
            raise ConnectionError("Failed to connect to CAN bus.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    # -----------------------------
    # low-level
    # -----------------------------
    def _require_bus(self):
        if self.bus is None or not self._is_connected:
            raise RuntimeError("CAN bus is not connected.")

    def _send(self, can_id: int, data: list[int]):
        self._require_bus()

        if len(data) != 8:
            raise ValueError(f"CAN data must be 8 bytes, got {len(data)} bytes.")

        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)
        log.debug("Sent to %s: %s", hex(can_id), data)

    def _send_recv(self, can_id: int, data: list[int], timeout: float = 1.0):
        """Send command and wait for matching response CAN ID."""
        self._require_bus()
        self._send(can_id, data)

        deadline = time.time() + timeout

        while time.time() < deadline:
            remain = deadline - time.time()
            if remain <= 0:
                break

            response = self.bus.recv(timeout=remain)
            if response is None:
                continue

            log.debug(
                "Received frame id=%s data=%s",
                hex(response.arbitration_id),
                list(response.data),
            )

            if response.arbitration_id == can_id:
                return response

        log.warning("Timeout waiting response from %s", hex(can_id))
        return None

    # -----------------------------
    # motor commands
    # -----------------------------
    def stop(self, can_id: int):
        data = [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._send(can_id, data)
        log.info("Stop command sent to %s", hex(can_id))

    def force_control(self, can_id: int, force: int):
        fc = int(force)
        data = [
            0xA1, 0x00, 0x00, 0x00,
            fc & 0xFF, (fc >> 8) & 0xFF,
            0x00, 0x00,
        ]
        self._send(can_id, data)
        log.info("Force control (%d) sent to %s", force, hex(can_id))

    def move_motor_to_angle(self, cmd: AngleCommand):
        angle_raw = int(cmd.angle * 1000)
        speed_raw = int(cmd.speed)

        data = [
            0xA4, 0x00,
            speed_raw & 0xFF, (speed_raw >> 8) & 0xFF,
            angle_raw & 0xFF, (angle_raw >> 8) & 0xFF,
            (angle_raw >> 16) & 0xFF, (angle_raw >> 24) & 0xFF,
        ]
        self._send(cmd.can_id, data)
        log.info(
            "Motor %s (%s) -> angle %.2f deg, speed %d",
            cmd.motor_name,
            hex(cmd.can_id),
            cmd.angle,
            cmd.speed,
        )

    def move_motors(self, commands: list[AngleCommand]):
        for cmd in commands:
            self.move_motor_to_angle(cmd)

    def increment_angle(self, can_id: int, angle_increment: float, max_speed: float):
        angle_raw = int(angle_increment * 1000)
        speed_raw = int(max_speed * 10)

        data = [
            0xA8, 0x00,
            speed_raw & 0xFF, (speed_raw >> 8) & 0xFF,
            angle_raw & 0xFF, (angle_raw >> 8) & 0xFF,
            (angle_raw >> 16) & 0xFF, (angle_raw >> 24) & 0xFF,
        ]
        self._send(can_id, data)
        log.info(
            "Increment angle %s: %.2f deg, max_speed %.2f",
            hex(can_id),
            angle_increment,
            max_speed,
        )

    def write_pid_gain(self, cmd: PidCommand):
        data = [
            0x31, 0x00,
            int(cmd.p_gain), int(cmd.i_gain),
            0x00, 0x00, 0x00, 0x00,
        ]
        self._send(cmd.can_id, data)
        log.info(
            "PID gains sent to %s (%s): P=%d, I=%d",
            cmd.motor_name,
            hex(cmd.can_id),
            cmd.p_gain,
            cmd.i_gain,
        )

    def write_acceleration(self, cmd: AccelCommand):
        accel = max(0, min(int(cmd.accel), 100))
        data = [
            0x34, 0x00, 0x00, 0x00,
            accel & 0xFF, (accel >> 8) & 0xFF,
            (accel >> 16) & 0xFF, (accel >> 24) & 0xFF,
        ]
        self._send(cmd.can_id, data)
        log.info("Acceleration %d sent to %s", accel, cmd.motor_name)

    def read_acceleration(self, can_id: int):
        data = [0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self._send_recv(can_id, data)

    def read_angle(self, can_id: int) -> int | None:
        data = [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        response = self._send_recv(can_id, data, timeout=1.0)
        if response is None:
            return None

        d = response.data
        if len(d) < 8:
            log.warning("Invalid angle response length from %s: %s", hex(can_id), list(d))
            return None

        motor_angle = (
            d[1]
            | (d[2] << 8)
            | (d[3] << 16)
            | (d[4] << 24)
            | (d[5] << 32)
            | (d[6] << 40)
            | (d[7] << 48)
        )
        return motor_angle


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    mc = MotorController()

    if not mc.connect():
        raise ConnectionError("Failed to connect to CAN bus.")

    try:
        cmds = [
            AngleCommand("left_joint1", 0, 360),
        ]
        mc.move_motors(cmds)

        angle = mc.read_angle(cmds[0].can_id)
        print("read_angle:", angle)

    finally:
        mc.disconnect()