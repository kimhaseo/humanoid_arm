import logging
import can
import serial.tools.list_ports

from config.motor_cmd import AngleCommand, AccelCommand, PidCommand

log = logging.getLogger(__name__)

# CAN adapter VID/PID (CANable / slcan)
_ADAPTER_VID = 0x16D0
_ADAPTER_PID = 0x117E


class MotorController:
    """CAN bus motor controller with context manager support.

    Usage:
        with MotorController() as mc:
            mc.move_motor_to_angle(AngleCommand("left_joint1", 90, 360))
    """

    def __init__(self, bitrate=1000000):
        self.bitrate = bitrate
        self.bus = None
        self.port = None

    # --- connection management ---

    def _find_can_port(self) -> bool:
        log.info("Searching for CAN adapter...")
        for port in serial.tools.list_ports.comports():
            log.debug("Port: %s, VID: %s, PID: %s", port.device, port.vid, port.pid)
            if port.vid == _ADAPTER_VID and port.pid == _ADAPTER_PID:
                log.info("Found CAN adapter: %s", port.device)
                self.port = port.device
                return True
        log.error("CAN adapter not found. Check device connection.")
        return False

    def connect(self) -> bool:
        if not self._find_can_port():
            return False
        try:
            self.bus = can.interface.Bus(
                interface="slcan", channel=self.port, bitrate=self.bitrate
            )
            log.info("CAN bus initialized on %s", self.port)
            return True
        except Exception as e:
            log.error("CAN bus init failed: %s", e)
            self.bus = None
            return False

    def disconnect(self):
        if self.bus:
            self.bus.shutdown()
            log.info("CAN bus disconnected.")

    def __enter__(self):
        if not self.connect():
            raise ConnectionError("Failed to connect to CAN bus.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    # --- low-level ---

    def _send(self, can_id, data):
        if not self.bus:
            raise RuntimeError("CAN bus is not connected.")
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        log.debug("Sent to %s: %s", hex(can_id), data)

    def _send_recv(self, can_id, data, timeout=1.0):
        self._send(can_id, data)
        response = self.bus.recv(timeout=timeout)
        log.debug("Response from %s: %s", hex(can_id), response)
        return response

    # --- motor commands ---

    def stop(self, can_id):
        self._send(can_id, [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        log.info("Stop command sent to %s", hex(can_id))

    def force_control(self, can_id, force):
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
        speed = cmd.speed
        data = [
            0xA4, 0x00,
            speed & 0xFF, (speed >> 8) & 0xFF,
            angle_raw & 0xFF, (angle_raw >> 8) & 0xFF,
            (angle_raw >> 16) & 0xFF, (angle_raw >> 24) & 0xFF,
        ]
        self._send(cmd.can_id, data)
        log.info("Motor %s -> angle %.2f deg", cmd.motor_name, cmd.angle)

    def move_motors(self, commands: list[AngleCommand]):
        for cmd in commands:
            self.move_motor_to_angle(cmd)

    def increment_angle(self, can_id, angle_increment, max_speed):
        angle_raw = int(angle_increment * 1000)
        speed_raw = int(max_speed * 10)
        data = [
            0xA8, 0x00,
            speed_raw & 0xFF, (speed_raw >> 8) & 0xFF,
            angle_raw & 0xFF, (angle_raw >> 8) & 0xFF,
            (angle_raw >> 16) & 0xFF, (angle_raw >> 24) & 0xFF,
        ]
        self._send(can_id, data)
        log.info("Increment angle %s: %.1f deg", hex(can_id), angle_increment)

    def write_pid_gain(self, cmd: PidCommand):
        data = [
            0x31, 0x00,
            int(cmd.p_gain), int(cmd.i_gain),
            0x00, 0x00, 0x00, 0x00,
        ]
        self._send(cmd.can_id, data)
        log.info("PID gains sent to %s: P=%d, I=%d", cmd.motor_name, cmd.p_gain, cmd.i_gain)

    def write_acceleration(self, cmd: AccelCommand):
        accel = min(int(cmd.accel), 100)
        data = [
            0x34, 0x00, 0x00, 0x00,
            accel & 0xFF, (accel >> 8) & 0xFF,
            (accel >> 16) & 0xFF, (accel >> 24) & 0xFF,
        ]
        self._send(cmd.can_id, data)
        log.info("Acceleration %d sent to %s", accel, cmd.motor_name)

    def read_acceleration(self, can_id):
        data = [0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self._send_recv(can_id, data)

    def read_angle(self, can_id):
        data = [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        response = self._send_recv(can_id, data)
        if response is None:
            return None
        d = response.data
        motor_angle = (
            d[1] | (d[2] << 8) | (d[3] << 16) | (d[4] << 24)
            | (d[5] << 32) | (d[6] << 40) | (d[7] << 48)
        )
        return motor_angle


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    with MotorController() as mc:
        cmds = [
            AngleCommand("left_joint1", 0, 360),
        ]
        mc.move_motors(cmds)
