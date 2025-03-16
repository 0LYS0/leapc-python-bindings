from conf import BLE_NAME, BLE_UUID, WRITE_CHAR_UUID
from read import read_FSR_values
import bluetooth

from micropython import const
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

# 블루투스 초기화
ble = bluetooth.BLE()
ble.active(True)

# 전역 상태 변수
is_connected = False
hr = None

# 연결 상태 관리 함수
def ble_irq(event, data):
    global is_connected, hr

    if event == _IRQ_CENTRAL_CONNECT:  # 연결됨
        print("블루투스 연결됨")
        is_connected = True

    elif event == _IRQ_CENTRAL_DISCONNECT:  # 연결 해제됨
        print("블루투스 연결 해제됨")
        is_connected = False
        if ble.active():  # BLE 활성 상태 확인
            advertise_ble()  # 연결 해제 시 광고 재시작
            
    elif event == _IRQ_GATTS_WRITE:
        conn_handle, attr_handle = data
        request = ble.gatts_read(attr_handle)
        if request == b'get_info':
            new_value = read_FSR_values().encode('utf-8')
            ble.gatts_write(hr, new_value)
            ble.gatts_notify(conn_handle, hr)


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
    global hr
    uuid = bluetooth.UUID(BLE_UUID)  # Requested service UUID
    # Combine WRITE, READ, and NOTIFY flags for the characteristic
    char_props = bluetooth.FLAG_WRITE | bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY
    char_def = (bluetooth.UUID(WRITE_CHAR_UUID), char_props)
    service = (uuid, (char_def,))
    hr = ble.gatts_register_services((service,))[0][0]
    ble.irq(ble_irq)
    advertise_ble()  # Start BLE advertising
    ble.config(mtu=256)


    
    

