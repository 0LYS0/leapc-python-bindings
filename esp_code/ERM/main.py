from machine import Pin
from time import sleep
import bluetooth

# board ID setting
from conf import BLE_NAME, BLE_UUID, WRITE_CHAR_UUID

# 핀 설정
pin_D2 = Pin(3, Pin.OUT)  # 진동 모터 제어 핀 1 (보드 상의 D2)
pin_D3 = Pin(4, Pin.OUT)  # 진동 모터 제어 핀 2 (보드 상의 D3)

# 블루투스 초기화
ble = bluetooth.BLE()
ble.active(True)

# 전역 상태 변수
is_connected = False

# 진동 모터 제어 함수
def motor_on_state(state):
    if state:  # 모터 켜기
        pin_D2.value(1)
        pin_D3.value(0)
    else:  # 모터 끄기
        pin_D2.value(0)
        pin_D3.value(0)

# 연결 상태 관리 함수
def ble_irq(event, data):
    global is_connected

    if event == 1:  # 연결됨
        print("블루투스 연결됨")
        is_connected = True
        motor_connect_notify()

    elif event == 2:  # 연결 해제됨
        motor_disconnect_notify()
        print("블루투스 연결 해제됨")
        is_connected = False
        if ble.active():  # BLE 활성 상태 확인
            advertise_ble()  # 연결 해제 시 광고 재시작

    elif event == 3:  # GATTS_WRITE 이벤트
        received_data = ble.gatts_read(data[1]).decode("utf-8").strip()
        print(f"수신 데이터: {received_data}")
        if received_data == "1":
            print(f"Motor On")
            motor_on_state(True)  # 모터 켜기
        elif received_data == "0":
            print(f"Motor Off")
            motor_on_state(False)  # 모터 끄기

# 블루투스 연결 알림
def motor_connect_notify():
    for _ in range(2):  # 0.5초 간격으로 2번 켜고 끄기
        motor_on_state(True)
        sleep(0.2)
        motor_on_state(False)
        sleep(0.1)
    motor_on_state(False)  # 이후 진동모터를 끈 상태 유지

# 블루투스 해제 알림
def motor_disconnect_notify():
    motor_on_state(True)
    sleep(1)  # 1초간 켜기
    motor_on_state(False)

# 블루투스 광고 시작
def advertise_ble():
    global BLE_NAME

    # 광고 데이터 생성
    name_bytes = bytes(BLE_NAME, 'utf-8')
    adv_data = bytearray(3 + len(name_bytes))
    adv_data[0] = len(name_bytes) + 1  # Length of this AD structure
    adv_data[1] = 0x09  # AD Type: Complete Local Name
    adv_data[2:] = name_bytes

    # 기존 광고 중지 후 새로운 광고 시작
    try:
        ble.gap_advertise(None)  # 기존 광고 중지
        ble.gap_advertise(100, adv_data)  # 새 광고 시작
        print(f"블루투스 광고 시작: {BLE_NAME}")
    except OSError as e:
        print(f"광고 시작 실패: {e}")

# 블루투스 서비스 설정
def setup_ble():
    uuid = bluetooth.UUID(BLE_UUID)  # 요청한 UUID
    write_char = (bluetooth.UUID(WRITE_CHAR_UUID), bluetooth.FLAG_WRITE)  # WRITE 특성 UUID
    service = (uuid, (write_char,))
    ble.gatts_register_services((service,))
    ble.irq(ble_irq)
    advertise_ble()  # 블루투스 광고 시작

# 초기화
setup_ble()

# 메인 루프
while True:
    # 블루투스 연결 상태를 계속 체크
    if is_connected:
        pass  # 연결된 동안 추가 작업이 필요하면 작성
    else:
        motor_on_state(False)  # 연결되지 않은 동안 모터 꺼짐 유지
    sleep(0.1)

