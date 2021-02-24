# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

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

_SENTENCE_PARAMS = {
    "GLL": ("DEGREES", "CHAR", "DEGREES", "CHAR", "FLOAT", "CHAR", "CHARN"),
    "RMC": (
        "FLOAT",
        "CHAR",
        "DEGREES",
        "CHAR",
        "DEGREES",
        "CHAR",
        "FLOAT",
        "FLOAT",
        "INT",
        "DEGREESN",
        "CHARN",
        "CHARN",
    ),
    "GGA": (
        "FLOAT",
        "DEGREES",
        "CHAR",
        "DEGREES",
        "CHAR",
        "INT",
        "INT",
        "FLOAT",
        "FLOAT",
        "STR",
        "FLOAT",
        "STR",
        "INTN",
        "STRN",
    ),
    "GSA": (
        "CHAR",
        "INT",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "FLOAT",
        "FLOAT",
        "FLOAT",
    ),
    "GSA_4_11": (  # NMEA 4.11 - Nov 2018
        "CHAR",
        "INT",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "INTN",
        "FLOAT",
        "FLOAT",
        "FLOAT",
        "STRN",
    ),
    "GSV7": (
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INTN",
    ),
    "GSV11": (
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INTN",
        "INT",
        "INT",
        "INT",
        "INTN",
    ),
    "GSV15": (
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INTN",
        "INT",
        "INT",
        "INT",
        "INTN",
        "INT",
        "INT",
        "INT",
        "INTN",
    ),
    "GSV19": (
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INT",
        "INTN",
        "INT",
        "INT",
        "INT",
        "INTN",
        "INT",
        "INT",
        "INT",
        "INTN",
        "INT",
        "INT",
        "INT",
        "INTN",
    ),
}


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


def _parse_talker(data_type):
    # Split the data_type into talker and sentence_type
    if data_type[0] == b"P":  # Proprietary codes
        return (data_type[:1], data_type[1:])

    return (data_type[:2], data_type[2:])


def _parse_data(sentence_type, data, debug):
    """Parse sentence data for the specified sentence type and
    return a list of parameters in the correct format, or return None.
    """

    if sentence_type not in _SENTENCE_PARAMS:
        # The sentence_type is unknown
        if debug:
            print(f"DEBUG:GPS: Unknown sentence type:{sentence_type}")
        return None

    param_types = _SENTENCE_PARAMS[sentence_type]

    if len(param_types) != len(data):
        # The expected number does not match the number of data items
        if debug:
            print(
                f"DEBUG:GPS: {sentence_type} len(param_types):{len(param_types)} "
                f"is not equal to len(data):{len(data)}"
            )
        return None

    params = []
    try:
        for i in range(len(data)):
            pti = param_types[i]
            di = data[i]
            if pti == "CHAR":
                # A single character
                if len(di) != 1:
                    return None
                params.append(di)
            elif pti == "CHARN":
                # A single character or Nothing
                if di is None or len(di) == 0:
                    params.append(None)
                elif len(di) != 1:
                    return None
                else:
                    params.append(di)
            elif pti == "DEGREES":
                # A number parseable as degrees
                params.append(_parse_degrees(di))
            elif pti == "DEGREESN":
                # A number parseable as degrees or Nothing
                if di is None or len(di) == 0:
                    params.append(None)
                else:
                    params.append(_parse_degrees(di))
            elif pti == "FLOAT":
                # A floating point number
                params.append(_parse_float(di))
            elif pti == "INT":
                # An integer
                params.append(_parse_int(di))
            elif pti == "INTN":
                # An integer or Nothing
                if di is None or len(di) == 0:
                    params.append(None)
                else:
                    params.append(_parse_int(di))
            elif pti == "STR":
                # A string
                params.append(di)
            elif pti == "STRN":
                # A string or Nothing
                if di is None or len(di) == 0:
                    params.append(None)
                else:
                    params.append(di)
            else:
                raise TypeError(f"Unexpected parameter type '{pti}'")
    except ValueError:
        # Something didn't parse, abort
        return None

    # Return the parsed data
    return params


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
        self.fix_quality = 0
        self.fix_quality_3d = 0
        self.satellites = None
        self.satellites_prev = None
        self.horizontal_dilution = None
        self.altitude_m = None
        self.height_geoid = None
        self.speed_knots = None
        self.track_angle_deg = None
        self._sats = None  # Temporary holder for information from GSV messages
        self.sats = None  # Completed information from GSV messages
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
        self._mode_indicator = None
        self._magnetic_variation = None
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
        (talker, sentence_type) = _parse_talker(data_type)

        # Check for all currently known GNSS talkers
        # GA - Galileo
        # GB - BeiDou Systems
        # GI - NavIC
        # GL - GLONASS
        # GP - GPS
        # GQ - QZSS
        # GN - GNSS / More than one of the above
        if talker not in (b"GA", b"GB", b"GI", b"GL", b"GP", b"GQ", b"GN"):
            # It's not a known GNSS source of data
            # Assume it's a valid packet anyway
            return True

        if sentence_type == b"GLL":  # Geographic position - Latitude/Longitude
            return self._parse_gll(args)
        elif sentence_type == b"RMC":  # Minimum location info
            return self._parse_rmc(args)
        elif sentence_type == b"GGA":  # 3D location fix
            return self._parse_gga(args)
        elif sentence_type == b"GSV":  # Satellites in view
            return self._parse_gsv(talker, args)
        elif sentence_type == b"GSA":  # GPS DOP and active satellites
            return self._parse_gsa(talker, args)
        else:
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

    def _update_timestamp_utc(self, time_utc, date=None):
        hours = time_utc // 10000
        mins = (time_utc // 100) % 100
        secs = time_utc % 100
        if date is None:
            if self.timestamp_utc is None:
                day, month, year = 0, 0, 0
            else:
                day = self.timestamp_utc.tm_mday
                month = self.timestamp_utc.tm_mon
                year = self.timestamp_utc.tm_year
        else:
            day = date // 10000
            month = (date // 100) % 100
            year = 2000 + date % 100

        self.timestamp_utc = time.struct_time(
            (year, month, day, hours, mins, secs, 0, 0, -1)
        )

    def _parse_gll(self, args):
        # GLL - Geographic Position - Latitude/Longitude

        data = args.split(",")
        if data is None or len(data) != 7:
            return False  # Unexpected number of params.
        data = _parse_data("GLL", data, self.debug)
        if data is None:
            return False  # Params didn't parse

        # Latitude
        self.latitude = data[0]
        if data[1].lower() == "s":
            self.latitude *= -1.0

        # Longitude
        self.longitude = data[2]
        if data[3].lower() == "w":
            self.longitude *= -1.0

        # UTC time of position
        self._update_timestamp_utc(int(data[4]))

        # Status Valid(A) or Invalid(V)
        self.isactivedata = data[5]

        # Parse FAA mode indicator
        self._mode_indicator = data[6]

        return True

    def _parse_rmc(self, args):
        # RMC - Recommended Minimum Navigation Information

        data = args.split(",")
        if data is None or len(data) != 12:
            return False  # Unexpected number of params.
        data = _parse_data("RMC", data, self.debug)
        if data is None:
            return False  # Params didn't parse

        # UTC time of position and date
        self._update_timestamp_utc(int(data[0]), data[8])

        # Status Valid(A) or Invalid(V)
        self.isactivedata = data[1]
        if data[1].lower() == "a":
            if self.fix_quality == 0:
                self.fix_quality = 1
        else:
            self.fix_quality = 0

        # Latitude
        self.latitude = data[2]
        if data[3].lower() == "s":
            self.latitude *= -1.0

        # Longitude
        self.longitude = data[4]
        if data[5].lower() == "w":
            self.longitude *= -1.0

        # Speed over ground, knots
        self.speed_knots = data[6]

        # Track made good, degrees true
        self.track_angle_deg = data[7]

        # Magnetic variation
        if data[9] is None or data[10] is None:
            self._magnetic_variation = None
        else:
            self._magnetic_variation = data[9]
            if data[10].lower() == "w":
                self._magnetic_variation *= -1.0

        # Parse FAA mode indicator
        self._mode_indicator = data[11]

        return True

    def _parse_gga(self, args):
        # GGA - Global Positioning System Fix Data

        data = args.split(",")
        if data is None or len(data) != 14:
            return False  # Unexpected number of params.
        data = _parse_data("GGA", data, self.debug)
        if data is None:
            return False  # Params didn't parse

        # UTC time of position
        self._update_timestamp_utc(int(data[0]))

        # Latitude
        self.latitude = data[1]
        if data[2].lower() == "s":
            self.latitude *= -1.0

        # Longitude
        self.longitude = data[3]
        if data[4].lower() == "w":
            self.longitude *= -1.0

        # GPS quality indicator
        # 0 - fix not available,
        # 1 - GPS fix,
        # 2 - Differential GPS fix (values above 2 are 2.3 features)
        # 3 - PPS fix
        # 4 - Real Time Kinematic
        # 5 - Float RTK
        # 6 - estimated (dead reckoning)
        # 7 - Manual input mode
        # 8 - Simulation mode
        self.fix_quality = data[5]

        # Number of satellites in use, 0 - 12
        self.satellites = data[6]

        # Horizontal dilution of precision
        self.horizontal_dilution = data[7]

        # Antenna altitude relative to mean sea level
        self.altitude_m = _parse_float(data[8])
        # data[9] - antenna altitude unit, always 'M' ???

        # Geoidal separation relative to WGS 84
        self.height_geoid = _parse_float(data[10])
        # data[11] - geoidal separation unit, always 'M' ???

        # data[12] - Age of differential GPS data, can be null
        # data[13] - Differential reference station ID, can be null

        return True

    def _parse_gsa(self, talker, args):
        # GSA - GPS DOP and active satellites

        data = args.split(",")
        if data is None or len(data) not in (17, 18):
            return False  # Unexpected number of params.
        if len(data) == 17:
            data = _parse_data("GSA", data, self.debug)
        else:
            data = _parse_data("GSA_4_11", data, self.debug)
        if data is None:
            return False  # Params didn't parse

        talker = talker.decode("ascii")

        # Selection mode: 'M' - manual, 'A' - automatic
        self.sel_mode = data[0]

        # Mode: 1 - no fix, 2 - 2D fix, 3 - 3D fix
        self.fix_quality_3d = data[1]

        satlist = list(filter(None, data[2:-4]))
        self.sat_prns = []
        for sat in satlist:
            self.sat_prns.append("{}{}".format(talker, sat))

        # PDOP, dilution of precision
        self.pdop = _parse_float(data[14])

        # HDOP, horizontal dilution of precision
        self.hdop = _parse_float(data[15])

        # VDOP, vertical dilution of precision
        self.vdop = _parse_float(data[16])

        # data[17] - System ID

        return True

    def _parse_gsv(self, talker, args):
        # GSV - Satellites in view
        # pylint: disable=too-many-branches

        data = args.split(",")
        if data is None or len(data) not in (7, 11, 15, 19):
            return False  # Unexpected number of params.
        data = _parse_data(
            {7: "GSV7", 11: "GSV11", 15: "GSV15", 19: "GSV19"}[len(data)],
            data,
            self.debug,
        )
        if data is None:
            return False  # Params didn't parse

        talker = talker.decode("ascii")

        # Number of messages
        self.total_mess_num = data[0]
        # Message number
        self.mess_num = data[1]
        # Number of satellites in view
        self.satellites = data[2]

        sat_tup = data[3:]

        satlist = []
        timestamp = time.monotonic()
        for i in range(len(sat_tup) // 4):
            j = i * 4
            value = (
                # Satellite number
                "{}{}".format(talker, sat_tup[0 + j]),
                # Elevation in degrees
                sat_tup[1 + j],
                # Azimuth in degrees
                sat_tup[2 + j],
                # signal-to-noise ratio in dB
                sat_tup[3 + j],
                # Timestamp
                timestamp,
            )
            satlist.append(value)

        if self._sats is None:
            self._sats = []
        for value in satlist:
            self._sats.append(value)

        if self.mess_num == self.total_mess_num:
            # Last part of GSV message
            if len(self._sats) == self.satellites:
                # Transfer received satellites to self.sats
                if self.sats is None:
                    self.sats = {}
                else:
                    # Remove all satellites which haven't
                    # been seen for 30 seconds
                    timestamp = time.monotonic()
                    old = []
                    for i in self.sats:
                        sat = self.sats[i]
                        if (timestamp - sat[4]) > 30:
                            old.append(i)
                    for i in old:
                        self.sats.pop(i)
                for sat in self._sats:
                    self.sats[sat[0]] = sat
            self._sats.clear()

        self.satellites_prev = self.satellites

        return True


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
