import serial, time

ser = serial.Serial("COM10", 9600)
time.sleep(5)  # 等 Arduino reset 完

while True:
    line = ser.readline().decode(errors="ignore").strip()
    print("Arduino:", line)
    if line == "READY":
        break

now_ms = int(time.time() * 1000)  # 傳毫秒 epoch
ser.write(f"{now_ms}\n".encode())
print("已傳送時間 (ms)：", now_ms)

reply = ser.readline().decode(errors="ignore").strip()
print("Arduino回應：", reply)
