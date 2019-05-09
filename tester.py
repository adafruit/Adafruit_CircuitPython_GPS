# Simple GPS module demonstration.
# Will wait for a fix and print a message every second with the current location
# and other details.
import time
import board
import busio

import adafruit_gps


RX = board.RX
TX = board.TX

uart = busio.UART(TX, RX, baudrate=9600, timeout=3)

gps = adafruit_gps.GPS(uart, debug=False)

# 0: GLL 
# 1: RMC
# 2: VTG
# 3: GGA
# 4: GSA
# 5: GSV
# 17: GPDZA
# 18: PMTKCHN
# gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on just minimum info (RMC only, location):
time.sleep(2)
gps.send_command(b'PMTK103*30')
time.sleep(10)
gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
time.sleep(0.2)
gps.send_command(b'PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on everything (not all of it is parsed!)
#gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

gps.send_command(b'PMTK220,100')

last_print = time.monotonic()
while True:
    gps.update()
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if not gps.has_fix:
            print("Waiting for fix.")
            continue
        print(gps._parse_sentence())
        continue
        print("=" * 40)
        if gps.total_mess_num is not None:
            print("Total number of messages: {}".format(gps.total_mess_num))

        if gps.mess_num is not None:
            print("Message number: {}".format(gps.mess_num))

        if gps.satellites is not None:
            print("Number of satellites: {}".format(gps.satellites))
        
        """
        try:
            a = [gps.gps0, gps.gps1, gps.gps2, gps.gps3, gps.gps4, gps.gps5]
        except 
        """

        try:
            if gps.gps0: 
                print("GPS number: {}".format(gps.gps0[0]))
                print("  Elevation: {} m".format(gps.gps0[1]))
                print("  Azimuth: {} deg".format(gps.gps0[2]))
                print("  SNR: {}".format(gps.gps0[3]))
            
            #print(gps.gps0)
        except AttributeError or TypeError:
            continue
