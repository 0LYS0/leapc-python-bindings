# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()
# from vib.py

from machine import Pin, PWM
from time import sleep
import bluetooth

# board ID setting
from conf import BLE_NAME, BLE_UUID, WRITE_CHAR_UUID


# 핀 설정

# 새끼손가락
pin_D1 = PWM(Pin(2), freq = 20000)
pin_D2 = Pin(3, Pin.OUT)
# 약지손가락
pin_D3 = PWM(Pin(4), freq = 20000)
pin_D4 = Pin(5, Pin.OUT)
# 중지손가락
pin_D5 = PWM(Pin(6), freq = 20000)
pin_D6 = Pin(1, Pin.OUT)
# 검지손가락
pin_D8 = PWM(Pin(7), freq = 20000)
pin_D7 = Pin(44, Pin.OUT)
# 엄지손가락
pin_D10 = PWM(Pin(9), freq = 20000)
pin_D9 = Pin(8, Pin.OUT)


# 블루투스 초기화
ble = bluetooth.BLE()
ble.active(True)


# 전역 상태 변수
is_connected = False


# 진동 모터 제어 함수
def motor_state(v1, v2, v3, v4, v5):
    MIN_PWM = 25000
    pin_D2.value(0)
    pin_D4.value(0)
    pin_D6.value(0)
    pin_D9.value(0)
    pin_D7.value(0)
    
    def control_motor(motor, speed): # 개별 속도 제어 함수  
        
        if speed == 0:  # 모터 정지
            motor.duty_u16(0)
        else:
            motor.duty_u16(65535) #Kick-start(처음에 100%로 주어 Starting Voltage 채우기)
            sleep(0.05)
        
            if speed < MIN_PWM:  # 최소 속도 유지
                motor.duty_u16(MIN_PWM)
            else:
                motor.duty_u16(speed)

    control_motor(pin_D10, v1) # 엄지
    control_motor(pin_D8, v2) # 검지
    control_motor(pin_D5, v3) # 중지
    control_motor(pin_D3, v4) # 약지
    control_motor(pin_D1, v5) # 새끼



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
        
        if not received_data:
            print("경고: 빈 데이터 수신됨. 무시합니다.")
            return  # 함수 종료
        
        try:
            # 쉼표(,)로 구분된 5개의 숫자를 받도록 처리
            received_values = received_data.split(",")  # "10000,20000,30000,40000,50000" → 리스트 변환
            
            if len(received_values) == 5:  # 숫자가 5개인지 확인
                pwm_values = []

                for val in received_values:
                    if val.strip().isdigit():  # 공백 제거 후 숫자인지 확인
                        pwm_value = int(val)

                        # 값이 유효한 범위인지 확인 (0~65535)
                        if 0 <= pwm_value <= 255:
                            pwm_values.append(int(pwm_value / 255 * 65535))
                        else:
                            print(f"유효하지 않은 값 ({pwm_value}), 0~65535 범위를 벗어남")
                            pwm_values.append(0)  # 잘못된 값이면 0으로 설정
                    else:
                        print(f"경고: 숫자가 아닌 데이터 수신 ('{val}'). 무시합니다.")
                        pwm_values.append(0)  # 잘못된 값이면 0으로 설정

                # 최종적으로 모터에 속도 적용
                print(f"모터 속도 설정: {pwm_values}")
                motor_state(*pwm_values)  # 5개의 값을 motor_state() 함수에 전달

            else:
                print(f"경고: 데이터 개수가 맞지 않음 (받은 데이터: {received_data})")

        except ValueError as e:
            print(f"ValueError 발생: {e}")
    

# 블루투스 연결 알림
def motor_connect_notify():
    for _ in range(2):  # 0.5초 간격으로 2번 켜고 끄기
        motor_state(65535, 65535, 65535, 65535, 65535)
        sleep(0.2)
        motor_state(65535, 65535, 65535, 65535, 65535)
        sleep(0.1)
    motor_state(0, 0, 0, 0, 0)  # 이후 진동모터를 끈 상태 유지


# 블루투스 해제 알림
def motor_disconnect_notify():
    motor_state(65535, 65535, 65535, 65535, 65535)
    sleep(1)  # 1초간 켜기
    motor_state(0, 0, 0, 0, 0)


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
    ble.config(mtu = 256)


# 초기화
setup_ble()


# 메인 루프
while True:
    # 블루투스 연결 상태를 계속 체크
    if is_connected:
        pass  # 연결된 동안 추가 작업이 필요하면 작성
    else:
        motor_state(0, 0, 0, 0, 0)  # 연결되지 않은 동안 모터 꺼짐 유지
    sleep(0.0001)


#     received_data = input("값을 입력하세요: ")
#     received_values = received_data.split(",")
#     v1 = int(received_values[0])
#     v2 = int(received_values[1])
#     v3 = int(received_values[2])
#     v4 = int(received_values[3])
#     v5 = int(received_values[4])
#     motor_state(v1, v2, v3, v4, v5)
#     sleep(10)




