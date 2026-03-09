import can


class DummyCanHandler:
    """COM 연결이 없을 때 사용하는 더미 핸들러 (시뮬레이션 모드)"""

    def __init__(self):
        print("[DUMMY] CAN 핸들러 생성됨 — 시뮬레이션 모드 (실제 모터 연결 없음)")

    def __call__(self, msg):
        print(f"[DUMMY] Received CAN message: {msg}")

    def send_message(self, can_id, data):
        print(f"[DUMMY] send_message: ID={hex(can_id)}, data={[hex(b) for b in data]}")

    def receive_message(self, timeout=1.0):
        # 8바이트 0x00 더미 응답 반환
        print(f"[DUMMY] receive_message → 더미 응답(8 x 0x00) 반환")
        return bytes(8)

    def close(self):
        print("[DUMMY] CAN bus 종료 (더미)")


def create_can_handler(channel="COM3", interface="slcan", bitrate=1000000):
    """실제 CAN 연결 시도 → 실패 시 DummyCanHandler로 자동 전환"""
    try:
        handler = CanHandler(channel=channel, interface=interface, bitrate=bitrate)
        print(f"[CAN] {channel} 연결 성공")
        return handler
    except Exception as e:
        print(f"[CAN] {channel} 연결 실패: {e}")
        print("[CAN] → 더미 모드(시뮬레이션)로 전환합니다")
        return DummyCanHandler()


class CanHandler:
    def __init__(self, channel="COM3", interface="slcan", bitrate=1000000):
        self.bus = can.interface.Bus(channel=channel, interface=interface, bitrate=bitrate)

    def __call__(self, msg):
        # 여기에서 수신된 메시지를 처리합니다.
        print(f"Received CAN message: {msg}")
        # 필요에 따라 메시지를 파싱하거나 추가 작업을 수행할 수 있습니다.

    def send_message(self, can_id, data):
        try:
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(message)
            # print(f"Sent CAN message: ID={can_id}, data={data}")
        except can.CanError as e:
            print(f"Error sending CAN message: ID={hex(can_id)}, {e}")
            raise

    def receive_message(self, timeout=1.0):
        try:
            response = self.bus.recv(timeout)  # 타임아웃 설정
            if response:
                # print(f"Received CAN message: {response}")
                return response
            else:
                print("No response received within the timeout period.")
                return None
        except Exception as e:
            print(f"Error receiving CAN message: {e}")

    def close(self):
        try:
            self.bus.shutdown()  # 버스를 종료하고 리소스를 해제
            print("CAN bus shut down properly.")
        except Exception as e:
            print(f"Error during CAN bus shutdown: {e}")