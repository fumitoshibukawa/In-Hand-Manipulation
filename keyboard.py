import serial
import keyboard  # pip install keyboard

ser = serial.Serial('/dev/ttyUSB0', 115200)  # デバイス名とボーレートを合わせる

print("操作キー: o=open, O=open(強), c=catch, g=grip, p=PD制御, s=stop")

while True:
    if keyboard.is_pressed('o'):
        ser.write(b'o')
    elif keyboard.is_pressed('O'):
        ser.write(b'O')
    elif keyboard.is_pressed('c'):
        ser.write(b'c')
    elif keyboard.is_pressed('g'):
        ser.write(b'g')
    elif keyboard.is_pressed('p'):
        ser.write(b'p')
    elif keyboard.is_pressed('s'):
        ser.write(b's')