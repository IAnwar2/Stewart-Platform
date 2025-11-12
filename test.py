# import serial, time
# PORT="/dev/tty.usbmodem1201"   # exact port
# BAUD=9600
# for ch in (1,2,3):             # try all three
#     with serial.Serial(PORT, BAUD, timeout=1) as ser:
#         time.sleep(2.5)
    
#         cmd = f"{ch} {15}\r\n"
#         print("Sending:", repr(cmd))
#         ser.write(cmd.encode("ascii"))
#         ser.flush()
#         time.sleep(0.6)

import serial
import time

# Open the serial connection (adjust the port for your system)
ser = serial.Serial('/dev/tty.usbmodem1201', 9600, timeout=1)
time.sleep(2)  # wait for Arduino reset

# Send a command
cmd = "2 15\n"
print(f"Sending: {repr(cmd)}")
ser.write(cmd.encode('ascii'))

# Optional: flush to make sure it's sent immediately
ser.flush()

# Now read back any response
time.sleep(0.1)  # give Arduino time to respond
response = ser.read_all().decode('ascii', errors='ignore')
print(f"Arduino replied: {response}")

# Clean up
ser.close()
