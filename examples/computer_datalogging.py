# Simple GPS datalogging demonstration for use with a computer like Linux/desktop.
# This actually doesn't even use the GPS library and instead just reads raw
# NMEA sentences from the GPS unit and dumps them to a file.

import serial  # pyserial is required

# Path to the file to log GPS data.  By default this will be appended to
# which means new lines are added at the end and all old data is kept.
# Change this path to point at the filename desired
LOG_FILE = 'gps.txt'  # Example for writing to local file gps.txt

# File more for opening the log file.  Mode 'ab' means append or add new lines
# to the end of the file rather than erasing it and starting over.  If you'd
# like to erase the file and start clean each time use the value 'wb' instead.
LOG_MODE = 'ab'

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# Update the serial port name to match the serial connection for the GPS!
uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=3000)

# Main loop just reads data from the GPS module and writes it back out to
# the output file while also printing to serial output.
with open(LOG_FILE, LOG_MODE) as outfile:
    while True:
        sentence = uart.readline()
        print(str(sentence, 'ascii').strip())
        outfile.write(sentence)
        outfile.flush()
