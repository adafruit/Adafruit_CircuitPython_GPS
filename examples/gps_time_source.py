# Simple script using GPS timestamps as RTC time source
# The GPS timestamps are available without a fix and keep the track of
# time while there is powersource (ie coin cell battery)

import time
import board
import busio
import rtc
import adafruit_gps

uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=30)

gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')

print("Set GPS as time source")
rtc.set_time_source(gps)

last_print = time.monotonic()
while True:

    gps.update()
    # Every second print out current time from GPS, RTC and time.localtime()
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        # Time & date from GPS informations
        print('Fix timestamp: {:02}/{:02}/{} {:02}:{:02}:{:02}'.format(
            gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
            gps.timestamp_utc.tm_mday,  # struct_time object that holds
            gps.timestamp_utc.tm_year,  # the fix time.  Note you might
            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
            gps.timestamp_utc.tm_min,   # month!
            gps.timestamp_utc.tm_sec))

        #Time & date from internal RTC
        print('RTC timestamp: {:02}/{:02}/{} {:02}:{:02}:{:02}'.format(
            rtc.RTC.datetime.tm_mon,
            rtc.RTC.datetime.tm_mday,
            rtc.RTC.datetime.tm_year,
            rtc.RTC.datetime.tm_hour,
            rtc.RTC.datetime.tm_min,
            rtc.RTC.datetime.tm_sec))

        #Time & date from time.localtime() function
        local_time = time.localtime()
        print("Local time: {:02}/{:02}/{} {:02}:{:02}:{:02}".format(
            local_time.tm_mon,
            local_time.tm_mday,
            local_time.tm_year,
            local_time.tm_hour,
            local_time.tm_min,
            local_time.tm_sec))
