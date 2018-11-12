# Simple GPS datalogging demonstration.
# This actually doesn't even use the GPS library and instead just reads raw
# NMEA sentences from the GPS unit and dumps them to a file on an SD card
# (recommended) or internal storage (be careful as only a few kilobytes to
# megabytes are available).  Before writing to internal storage you MUST
# carefully follow the steps in this guide to enable writes to the internal
# filesystem:
#  https://learn.adafruit.com/adafruit-ultimate-gps-featherwing/circuitpython-library
import board
import busio


# Path to the file to log GPS data.  By default this will be appended to
# which means new lines are added at the end and all old data is kept.
# Change this path to point at internal storage (like '/gps.txt') or SD
# card mounted storage ('/sd/gps.txt') as desired.
LOG_FILE = '/gps.txt'  # Example for writing to internal path /gps.txt
#LOG_FILE = '/sd/gps.txt'     # Example for writing to SD card path /sd/gps.txt

# File more for opening the log file.  Mode 'ab' means append or add new lines
# to the end of the file rather than erasing it and starting over.  If you'd
# like to erase the file and start clean each time use the value 'wb' instead.
LOG_MODE = 'ab'

# Define RX and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
RX = board.RX
TX = board.TX

# If writing to SD card customize and uncomment these lines to import the
# necessary library and initialize the SD card:
#SD_CS_PIN = board.SD_CS  # CS for SD card (SD_CS is for Feather Adalogger)
#import adafruit_sdcard
#spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
#sd_cs = digitalio.DigitalInOut(SD_CS_PIN)
#sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
#vfs = storage.VfsFat(sdcard)
#storage.mount(vfs, '/sd')  # Mount SD card under '/sd' path in filesystem.

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
uart = busio.UART(TX, RX, baudrate=9600, timeout=3000)

# Main loop just reads data from the GPS module and writes it back out to
# the output file while also printing to serial output.
with open(LOG_FILE, LOG_MODE) as outfile:
    while True:
        sentence = uart.readline()
        print(str(sentence, 'ascii').strip())
        outfile.write(sentence)
        outfile.flush()
