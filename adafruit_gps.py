# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_gps`
====================================================

GPS parsing module.  Can parse simple NMEA data sentences from serial GPS
modules to read latitude, longitude, and more.

* Author(s): Tony DiCola

Implementation Notes
--------------------

**Hardware:**

* Adafruit `Ultimate GPS Breakout <https://www.adafruit.com/product/746>`_
* Adafruit `Ultimate GPS FeatherWing <https://www.adafruit.com/product/3133>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases

"""
import time
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_GPS.git"


_GPSI2C_DEFAULT_ADDRESS = const(0x10)

# Internal helper parsing functions.
# These handle input that might be none or null and return none instead of
# throwing errors.
def _parse_degrees(nmea_data):
    # Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
    # Where ddd is the degrees, mm.mmmm is the minutes.
    if nmea_data is None or len(nmea_data) < 3:
        return None
    raw = float(nmea_data)
    deg = raw // 100
    minutes = raw % 100
    return deg + minutes / 60


def _parse_int(nmea_data):
    if nmea_data is None or nmea_data == "":
        return None
    return int(nmea_data)


def _parse_float(nmea_data):
    if nmea_data is None or nmea_data == "":
        return None
    return float(nmea_data)


def _parse_str(nmea_data):
    if nmea_data is None or nmea_data == "":
        return None
    return str(nmea_data)


# lint warning about too many attributes disabled
# pylint: disable-msg=R0902


class GPS:
    """GPS parsing module.  Can parse simple NMEA data sentences from serial
    GPS modules to read latitude, longitude, and more.
    """

    def __init__(self, uart, debug=False):
        self._uart = uart
        # Initialize null starting values for GPS attributes.
        self.timestamp_utc = None
        self.latitude = None
        self.longitude = None
        self.fix_quality = None
        self.fix_quality_3d = None
        self.satellites = None
        self.satellites_prev = None
        self.horizontal_dilution = None
        self.altitude_m = None
        self.height_geoid = None
        self.speed_knots = None
        self.track_angle_deg = None
        self.sats = None
        self.isactivedata = None
        self.true_track = None
        self.mag_track = None
        self.sat_prns = None
        self.sel_mode = None
        self.pdop = None
        self.hdop = None
        self.vdop = None
        self.total_mess_num = None
        self.mess_num = None
        self._raw_sentence = None
        self.debug = debug

    def update(self):
        """Check for updated data from the GPS module and process it
        accordingly.  Returns True if new data was processed, and False if
        nothing new was received.
        """
        # Grab a sentence and check its data type to call the appropriate
        # parsing function.
        try:
            sentence = self._parse_sentence()
        except UnicodeError:
            return None
        if sentence is None:
            return False
        if self.debug:
            print(sentence)
        data_type, args = sentence
        data_type = bytes(data_type.upper(), "ascii")
        # return sentence
        if data_type in (
            b"GPGLL",
            b"GNGGL",
        ):  # GLL, Geographic Position – Latitude/Longitude
            self._parse_gpgll(args)
        elif data_type in (b"GPRMC", b"GNRMC"):  # RMC, minimum location info
            self._parse_gprmc(args)
        elif data_type in (b"GPGGA", b"GNGGA"):  # GGA, 3d location fix
            self._parse_gpgga(args)
        return True

    def send_command(self, command, add_checksum=True):
        """Send a command string to the GPS.  If add_checksum is True (the
        default) a NMEA checksum will automatically be computed and added.
        Note you should NOT add the leading $ and trailing * to the command
        as they will automatically be added!
        """
        self.write(b"$")
        self.write(command)
        if add_checksum:
            checksum = 0
            for char in command:
                checksum ^= char
            self.write(b"*")
            self.write(bytes("{:02x}".format(checksum).upper(), "ascii"))
        self.write(b"\r\n")

    @property
    def has_fix(self):
        """True if a current fix for location information is available."""
        return self.fix_quality is not None and self.fix_quality >= 1

    @property
    def has_3d_fix(self):
        """Returns true if there is a 3d fix available.
        use has_fix to determine if a 2d fix is available,
        passing it the same data"""
        return self.fix_quality_3d is not None and self.fix_quality_3d >= 2

    @property
    def datetime(self):
        """Return struct_time object to feed rtc.set_time_source() function"""
        return self.timestamp_utc

    @property
    def nmea_sentence(self):
        """Return raw_sentence which is the raw NMEA sentence read from the GPS"""
        return self._raw_sentence

    def read(self, num_bytes):
        """Read up to num_bytes of data from the GPS directly, without parsing.
        Returns a bytearray with up to num_bytes or None if nothing was read"""
        return self._uart.read(num_bytes)

    def write(self, bytestr):
        """Write a bytestring data to the GPS directly, without parsing
        or checksums"""
        return self._uart.write(bytestr)

    @property
    def in_waiting(self):
        """Returns number of bytes available in UART read buffer"""
        return self._uart.in_waiting

    def readline(self):
        """Returns a newline terminated bytearray, must have timeout set for
        the underlying UART or this will block forever!"""
        return self._uart.readline()

    def _read_sentence(self):
        # Parse any NMEA sentence that is available.
        # pylint: disable=len-as-condition
        # This needs to be refactored when it can be tested.

        # Only continue if we have at least 32 bytes in the input buffer
        if self.in_waiting < 32:
            return None

        sentence = self.readline()
        if sentence is None or sentence == b"" or len(sentence) < 1:
            return None
        try:
            sentence = str(sentence, "ascii").strip()
        except UnicodeError:
            return None
        # Look for a checksum and validate it if present.
        if len(sentence) > 7 and sentence[-3] == "*":
            # Get included checksum, then calculate it and compare.
            expected = int(sentence[-2:], 16)
            actual = 0
            for i in range(1, len(sentence) - 3):
                actual ^= ord(sentence[i])
            if actual != expected:
                return None  # Failed to validate checksum.

            # copy the raw sentence
            self._raw_sentence = sentence

            return sentence
        # At this point we don't have a valid sentence
        return None

    def _parse_sentence(self):
        sentence = self._read_sentence()

        # sentence is a valid NMEA with a valid checksum
        if sentence is None:
            return None

        # Remove checksum once validated.
        sentence = sentence[:-3]
        # Parse out the type of sentence (first string after $ up to comma)
        # and then grab the rest as data within the sentence.
        delimiter = sentence.find(",")
        if delimiter == -1:
            return None  # Invalid sentence, no comma after data type.
        data_type = sentence[1:delimiter]
        return (data_type, sentence[delimiter + 1 :])

    def _parse_gpgll(self, args):
        data = args.split(",")
        if data is None or data[0] is None:
            return  # Unexpected number of params.

        # Parse latitude and longitude.
        self.latitude = _parse_degrees(data[0])
        if self.latitude is not None and data[1] is not None and data[1].lower() == "s":
            self.latitude *= -1.0
        self.longitude = _parse_degrees(data[2])
        if (
            self.longitude is not None
            and data[3] is not None
            and data[3].lower() == "w"
        ):
            self.longitude *= -1.0
        time_utc = int(_parse_int(float(data[4])))
        if time_utc is not None:
            hours = time_utc // 10000
            mins = (time_utc // 100) % 100
            secs = time_utc % 100
            # Set or update time to a friendly python time struct.
            if self.timestamp_utc is not None:
                self.timestamp_utc = time.struct_time(
                    (0, 0, 0, hours, mins, secs, 0, 0, -1)
                )
            else:
                self.timestamp_utc = time.struct_time(
                    (0, 0, 0, hours, mins, secs, 0, 0, -1)
                )
        # Parse data active or void
        self.isactivedata = _parse_str(data[5])

    def _parse_gprmc(self, args):
        # Parse the arguments (everything after data type) for NMEA GPRMC
        # minimum location fix sentence.
        data = args.split(",")
        if data is None or len(data) < 11 or data[0] is None:
            return  # Unexpected number of params.
        # Parse fix time.
        time_utc = int(_parse_float(data[0]))
        if time_utc is not None:
            hours = time_utc // 10000
            mins = (time_utc // 100) % 100
            secs = time_utc % 100
            # Set or update time to a friendly python time struct.
            if self.timestamp_utc is not None:
                self.timestamp_utc = time.struct_time(
                    (
                        self.timestamp_utc.tm_year,
                        self.timestamp_utc.tm_mon,
                        self.timestamp_utc.tm_mday,
                        hours,
                        mins,
                        secs,
                        0,
                        0,
                        -1,
                    )
                )
            else:
                self.timestamp_utc = time.struct_time(
                    (0, 0, 0, hours, mins, secs, 0, 0, -1)
                )
        # Parse status (active/fixed or void).
        status = data[1]
        self.fix_quality = 0
        if status is not None and status.lower() == "a":
            self.fix_quality = 1
        # Parse latitude and longitude.
        self.latitude = _parse_degrees(data[2])
        if self.latitude is not None and data[3] is not None and data[3].lower() == "s":
            self.latitude *= -1.0
        self.longitude = _parse_degrees(data[4])
        if (
            self.longitude is not None
            and data[5] is not None
            and data[5].lower() == "w"
        ):
            self.longitude *= -1.0
        # Parse out speed and other simple numeric values.
        self.speed_knots = _parse_float(data[6])
        self.track_angle_deg = _parse_float(data[7])
        # Parse date.
        if data[8] is not None and len(data[8]) == 6:
            day = int(data[8][0:2])
            month = int(data[8][2:4])
            year = 2000 + int(data[8][4:6])  # Y2k bug, 2 digit year assumption.
            # This is a problem with the NMEA
            # spec and not this code.
            if self.timestamp_utc is not None:
                # Replace the timestamp with an updated one.
                # (struct_time is immutable and can't be changed in place)
                self.timestamp_utc = time.struct_time(
                    (
                        year,
                        month,
                        day,
                        self.timestamp_utc.tm_hour,
                        self.timestamp_utc.tm_min,
                        self.timestamp_utc.tm_sec,
                        0,
                        0,
                        -1,
                    )
                )
            else:
                # Time hasn't been set so create it.
                self.timestamp_utc = time.struct_time(
                    (year, month, day, 0, 0, 0, 0, 0, -1)
                )

    def _parse_gpgga(self, args):
        # Parse the arguments (everything after data type) for NMEA GPGGA
        # 3D location fix sentence.
        data = args.split(",")
        if data is None or len(data) != 14:
            return  # Unexpected number of params.
        # Parse fix time.
        time_utc = int(_parse_float(data[0]))
        if time_utc is not None:
            hours = time_utc // 10000
            mins = (time_utc // 100) % 100
            secs = time_utc % 100
            # Set or update time to a friendly python time struct.
            if self.timestamp_utc is not None:
                self.timestamp_utc = time.struct_time(
                    (
                        self.timestamp_utc.tm_year,
                        self.timestamp_utc.tm_mon,
                        self.timestamp_utc.tm_mday,
                        hours,
                        mins,
                        secs,
                        0,
                        0,
                        -1,
                    )
                )
            else:
                self.timestamp_utc = time.struct_time(
                    (0, 0, 0, hours, mins, secs, 0, 0, -1)
                )
        # Parse latitude and longitude.
        self.latitude = _parse_degrees(data[1])
        if self.latitude is not None and data[2] is not None and data[2].lower() == "s":
            self.latitude *= -1.0
        self.longitude = _parse_degrees(data[3])
        if (
            self.longitude is not None
            and data[4] is not None
            and data[4].lower() == "w"
        ):
            self.longitude *= -1.0
        # Parse out fix quality and other simple numeric values.
        self.fix_quality = _parse_int(data[5])
        self.satellites = _parse_int(data[6])
        self.horizontal_dilution = _parse_float(data[7])
        self.altitude_m = _parse_float(data[8])
        self.height_geoid = _parse_float(data[10])

    def _parse_gpgsa(self, args):
        data = args.split(",")
        if data is None:
            return  # Unexpected number of params

        # Parse selection mode
        self.sel_mode = _parse_str(data[0])
        # Parse 3d fix
        self.fix_quality_3d = _parse_int(data[1])
        satlist = list(filter(None, data[2:-4]))
        self.sat_prns = {}
        for i, sat in enumerate(satlist, 1):
            self.sat_prns["gps{}".format(i)] = _parse_int(sat)

        # Parse PDOP, dilution of precision
        self.pdop = _parse_float(data[-3])
        # Parse HDOP, horizontal dilution of precision
        self.hdop = _parse_float(data[-2])
        # Parse VDOP, vertical dilution of precision
        self.vdop = _parse_float(data[-1])

    def _parse_gpgsv(self, args):
        # Parse the arguments (everything after data type) for NMEA GPGGA
        # 3D location fix sentence.
        data = args.split(",")
        if data is None:
            return  # Unexpected number of params.

        # Parse number of messages
        self.total_mess_num = _parse_int(data[0])  # Total number of messages
        # Parse message number
        self.mess_num = _parse_int(data[1])  # Message number
        # Parse number of satellites in view
        self.satellites = _parse_int(data[2])  # Number of satellites

        if len(data) < 3:
            return

        sat_tup = data[3:]

        satdict = {}
        for i in range(len(sat_tup) / 4):
            j = i * 4
            key = "gps{}".format(i + (4 * (self.mess_num - 1)))
            satnum = _parse_int(sat_tup[0 + j])  # Satellite number
            satdeg = _parse_int(sat_tup[1 + j])  # Elevation in degrees
            satazim = _parse_int(sat_tup[2 + j])  # Azimuth in degrees
            satsnr = _parse_int(sat_tup[3 + j])  # signal-to-noise ratio in dB
            value = (satnum, satdeg, satazim, satsnr)
            satdict[key] = value

        if self.sats is None:
            self.sats = {}
        for satnum in satdict:
            self.sats[satnum] = satdict[satnum]

        try:
            if self.satellites < self.satellites_prev:
                for i in self.sats:
                    try:
                        if int(i[-2]) >= self.satellites:
                            del self.sats[i]
                    except ValueError:
                        if int(i[-1]) >= self.satellites:
                            del self.sats[i]
        except TypeError:
            pass
        self.satellites_prev = self.satellites


class GPS_GtopI2C(GPS):
    """GTop-compatible I2C GPS parsing module.  Can parse simple NMEA data
    sentences from an I2C-capable GPS module to read latitude, longitude, and more.
    """

    def __init__(
        self, i2c_bus, *, address=_GPSI2C_DEFAULT_ADDRESS, debug=False, timeout=5
    ):
        import adafruit_bus_device.i2c_device as i2c_device  # pylint: disable=import-outside-toplevel

        super().__init__(None, debug)  # init the parent with no UART
        self._i2c = i2c_device.I2CDevice(i2c_bus, address)
        self._lastbyte = None
        self._charbuff = bytearray(1)
        self._internalbuffer = []
        self._timeout = timeout

    def read(self, num_bytes=1):
        """Read up to num_bytes of data from the GPS directly, without parsing.
        Returns a bytearray with up to num_bytes or None if nothing was read"""
        result = []
        for _ in range(num_bytes):
            with self._i2c as i2c:
                # we read one byte at a time, verify it isnt part of a string of
                # 'stuffed' newlines and then append to our result array for byteification
                i2c.readinto(self._charbuff)
                char = self._charbuff[0]
                if (char == ord("\n")) and (self._lastbyte != ord("\r")):
                    continue  # skip duplicate \n's!
                result.append(char)
                self._lastbyte = char  # keep track of the last character approved
        return bytearray(result)

    def write(self, bytestr):
        """Write a bytestring data to the GPS directly, without parsing
        or checksums"""
        with self._i2c as i2c:
            i2c.write(bytestr)

    @property
    def in_waiting(self):
        """Returns number of bytes available in UART read buffer, always 32
        since I2C does not have the ability to know how much data is available"""
        return 32

    def readline(self):
        """Returns a newline terminated bytearray, must have timeout set for
        the underlying UART or this will block forever!"""
        timeout = time.monotonic() + self._timeout
        while timeout > time.monotonic():
            # check if our internal buffer has a '\n' termination already
            if self._internalbuffer and (self._internalbuffer[-1] == ord("\n")):
                break
            char = self.read(1)
            if not char:
                continue
            self._internalbuffer.append(char[0])
            # print(bytearray(self._internalbuffer))
        if self._internalbuffer and self._internalbuffer[-1] == ord("\n"):
            ret = bytearray(self._internalbuffer)
            self._internalbuffer = []  # reset the buffer to empty
            return ret
        return None  # no completed data yet
