# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2021 James Carr
#
# SPDX-License-Identifier: MIT

"""
`adafruit_gps`
====================================================

GPS parsing module.  Can parse simple NMEA data sentences from serial GPS
modules to read latitude, longitude, and more.

* Author(s): Tony DiCola, James Carr

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

try:
    from typing import Optional, Tuple, List
    from typing_extensions import Literal
    from circuitpython_typing import ReadableBuffer
    from busio import UART, I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_GPS.git"


_GPSI2C_DEFAULT_ADDRESS = const(0x10)

_GLL = 0
_RMC = 1
_GGA = 2
_GSA = 3
_GSA_4_11 = 4
_GSV7 = 5
_GSV11 = 6
_GSV15 = 7
_GSV19 = 8
_RMC_4_1 = 9
_ST_MIN = _GLL
_ST_MAX = _RMC_4_1

_SENTENCE_PARAMS = (
    # 0 - _GLL
    "dcdcscC",
    # 1 - _RMC
    "scdcdcffsDCC",
    # 2 - _GGA
    "sdcdciiffsfsIS",
    # 3 - _GSA
    "ciIIIIIIIIIIIIfff",
    # 4 - _GSA_4_11
    "ciIIIIIIIIIIIIfffS",
    # 5 - _GSV7
    "iiiiiiI",
    # 6 - _GSV11
    "iiiiiiIiiiI",
    # 7 - _GSV15
    "iiiiiiIiiiIiiiI",
    # 8 - _GSV19
    "iiiiiiIiiiIiiiIiiiI",
    # 9 - _RMC_4_1
    "scdcdcffsDCCC",
)


# Internal helper parsing functions.
# These handle input that might be none or null and return none instead of
# throwing errors.
def _parse_degrees(nmea_data: str) -> int:
    # Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
    # Where ddd is the degrees, mm.mmmm is the minutes.
    if nmea_data is None or len(nmea_data) < 3:
        return None
    # To avoid losing precision handle degrees and minutes separately
    # Return the final value as an integer. Further functions can parse
    # this into a float or separate parts to retain the precision
    raw = nmea_data.split(".")
    degrees = int(raw[0]) // 100 * 1000000  # the ddd
    minutes = int(raw[0]) % 100  # the mm.
    minutes += int(f"{raw[1][:4]:0<4}") / 10000
    minutes = int(minutes / 60 * 1000000)
    return degrees + minutes


def _parse_int(nmea_data: str) -> int:
    if nmea_data is None or nmea_data == "":
        return None
    return int(nmea_data)


def _parse_float(nmea_data: str) -> float:
    if nmea_data is None or nmea_data == "":
        return None
    return float(nmea_data)


def _parse_str(nmea_data: str) -> str:
    if nmea_data is None or nmea_data == "":
        return None
    return str(nmea_data)


def _read_degrees(data: List[float], index: int, neg: str) -> float:
    # This function loses precision with float32
    x = data[index] / 1000000
    if data[index + 1].lower() == neg:
        x *= -1.0
    return x


def _read_int_degrees(data: List[float], index: int, neg: str) -> Tuple[int, float]:
    deg = data[index] // 1000000
    minutes = data[index] % 1000000 / 10000
    if data[index + 1].lower() == neg:
        deg *= -1
    return (deg, minutes)


def _parse_talker(data_type: bytes) -> Tuple[bytes, bytes]:
    # Split the data_type into talker and sentence_type
    if data_type[:1] == b"P":  # Proprietary codes
        return (data_type[:1], data_type[1:])

    return (data_type[:2], data_type[2:])


def _parse_data(sentence_type: int, data: List[str]) -> Optional[List]:
    """Parse sentence data for the specified sentence type and
    return a list of parameters in the correct format, or return None.
    """
    # pylint: disable=too-many-branches

    if not _ST_MIN <= sentence_type <= _ST_MAX:
        # The sentence_type is unknown
        return None

    param_types = _SENTENCE_PARAMS[sentence_type]

    if len(param_types) != len(data):
        # The expected number does not match the number of data items
        return None

    params = []
    try:
        for i, dti in enumerate(data):
            pti = param_types[i]
            len_dti = len(dti)
            nothing = dti is None or len_dti == 0
            if pti == "c":
                # A single character
                if len_dti != 1:
                    return None
                params.append(dti)
            elif pti == "C":
                # A single character or Nothing
                if nothing:
                    params.append(None)
                elif len_dti != 1:
                    return None
                else:
                    params.append(dti)
            elif pti == "d":
                # A number parseable as degrees
                params.append(_parse_degrees(dti))
            elif pti == "D":
                # A number parseable as degrees or Nothing
                if nothing:
                    params.append(None)
                else:
                    params.append(_parse_degrees(dti))
            elif pti == "f":
                # A floating point number
                params.append(_parse_float(dti))
            elif pti == "i":
                # An integer
                params.append(_parse_int(dti))
            elif pti == "I":
                # An integer or Nothing
                if nothing:
                    params.append(None)
                else:
                    params.append(_parse_int(dti))
            elif pti == "s":
                # A string
                params.append(dti)
            elif pti == "S":
                # A string or Nothing
                if nothing:
                    params.append(None)
                else:
                    params.append(dti)
            else:
                raise TypeError(f"GPS: Unexpected parameter type '{pti}'")
    except ValueError:
        # Something didn't parse, abort
        return None

    # Return the parsed data
    return params


# pylint: disable-msg=too-many-instance-attributes
class GPS:
    """GPS parsing module.  Can parse simple NMEA data sentences from serial
    GPS modules to read latitude, longitude, and more.
    """

    # lint warning about too many statements disabled
    # pylint: disable-msg=R0915
    def __init__(self, uart: UART, debug: bool = False) -> None:
        self._uart = uart
        # Initialize null starting values for GPS attributes.
        self.timestamp_utc = None
        """Timestamp in UTC"""
        self.latitude = None
        """Degrees latitude"""
        self.latitude_degrees = None
        """Degrees component of latitude measurement"""
        self.latitude_minutes = None  # Use for full precision minutes
        """Minutes component of latitude measurement"""
        self.longitude = None
        """Degrees longitude"""
        self.longitude_degrees = None
        """Degrees component of longitude measurement"""
        self.longitude_minutes = None  # Use for full precision minutes
        """Minutes component of longitude measurement"""
        self.fix_quality = 0
        """
        GPS quality indicator

            |   0 - fix not available
            |   1 - GPS fix
            |   2 - Differential GPS fix (values above 2 are 2.3 features)
            |   3 - PPS fix
            |   4 - Real Time Kinematic
            |   5 - Float RTK
            |   6 - estimated (dead reckoning)
            |   7 - Manual input mode
            |   8 - Simulation mode
        """
        self.fix_quality_3d = 0
        """
        The type of fix for a reading

            |    1 - no fix
            |    2 - 2D fix
            |    3 - 3D fix
        """
        self.satellites = None
        """The number of satellites in use, 0 - 12"""
        self.satellites_prev = None
        """The number of satellites in use from the previous data point, 0 - 12"""
        self.horizontal_dilution = None
        """Horizontal dilution of precision (GGA)"""
        self.altitude_m = None
        """Antenna altitude relative to mean sea level"""
        self.height_geoid = None
        """Geoidal separation relative to WGS 84"""
        self.speed_knots = None
        """Ground speed in knots"""
        self.track_angle_deg = None
        """Track angle in degrees"""
        self._sats = None  # Temporary holder for information from GSV messages
        self.sats = None
        """Information from GSV messages"""
        self.isactivedata = None
        """Status Valid(A) or Invalid(V)"""
        self.sat_prns = None
        """Satellite pseudorandom noise code"""
        self.sel_mode = None
        """
        Selection mode

            |   'M' - manual
            |   'A' - automatic
        """
        self.pdop = None
        """Dilution of precision"""
        self.hdop = None
        """Horizontal dilution of precision (GSA)"""
        self.vdop = None
        """Vertical dilution of precision"""
        self.total_mess_num = None
        """Number of messages"""
        self.mess_num = None
        """Message number"""
        self._raw_sentence = None
        self._mode_indicator = None
        self._magnetic_variation = None
        self.debug = debug
        """Toggles debug mode. When True, prints the incoming data sentence to the console"""

    def update(self) -> bool:
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
        if len(data_type) < 5:
            return False
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

        result = True
        args = args.split(",")
        if sentence_type == b"GLL":  # Geographic position - Latitude/Longitude
            result = self._parse_gll(args)
        elif sentence_type == b"RMC":  # Minimum location info
            result = self._parse_rmc(args)
        elif sentence_type == b"GGA":  # 3D location fix
            result = self._parse_gga(args)
        elif sentence_type == b"GSV":  # Satellites in view
            result = self._parse_gsv(talker, args)
        elif sentence_type == b"GSA":  # GPS DOP and active satellites
            result = self._parse_gsa(talker, args)

        return result

    def send_command(self, command: bytes, add_checksum: bool = True) -> None:
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
    def has_fix(self) -> bool:
        """True if a current fix for location information is available."""
        return self.fix_quality is not None and self.fix_quality >= 1

    @property
    def has_3d_fix(self) -> bool:
        """Returns true if there is a 3d fix available.
        use has_fix to determine if a 2d fix is available,
        passing it the same data"""
        return self.fix_quality_3d is not None and self.fix_quality_3d >= 2

    @property
    def datetime(self) -> Optional[time.struct_time]:
        """Return struct_time object to feed rtc.set_time_source() function"""
        return self.timestamp_utc

    @property
    def nmea_sentence(self) -> Optional[str]:
        """Return raw_sentence which is the raw NMEA sentence read from the GPS"""
        return self._raw_sentence

    def read(self, num_bytes: Optional[int]) -> Optional[bytes]:
        """Read up to num_bytes of data from the GPS directly, without parsing.
        Returns a bytestring with up to num_bytes or None if nothing was read"""
        return self._uart.read(num_bytes)

    def write(self, bytestr: ReadableBuffer) -> Optional[int]:
        """Write a bytestring data to the GPS directly, without parsing
        or checksums"""
        return self._uart.write(bytestr)

    @property
    def in_waiting(self) -> int:
        """Returns number of bytes available in UART read buffer"""
        return self._uart.in_waiting

    def readline(self) -> Optional[bytes]:
        """Returns a newline terminated bytestring, must have timeout set for
        the underlying UART or this will block forever!"""
        return self._uart.readline()

    def _read_sentence(self) -> Optional[str]:
        # Parse any NMEA sentence that is available.
        # pylint: disable=len-as-condition
        # This needs to be refactored when it can be tested.

        # Only continue if we have at least 11 bytes in the input buffer
        if self.in_waiting < 11:
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

    def _parse_sentence(self) -> Optional[Tuple[str, str]]:
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

    def _update_timestamp_utc(self, time_utc: str, date: Optional[str] = None) -> None:
        hours = int(time_utc[0:2])
        mins = int(time_utc[2:4])
        secs = int(time_utc[4:6])
        if date is None:
            if self.timestamp_utc is None:
                day, month, year = 0, 0, 0
            else:
                day = self.timestamp_utc.tm_mday
                month = self.timestamp_utc.tm_mon
                year = self.timestamp_utc.tm_year
        else:
            day = int(date[0:2])
            month = int(date[2:4])
            year = 2000 + int(date[4:6])

        self.timestamp_utc = time.struct_time(
            (year, month, day, hours, mins, secs, 0, 0, -1)
        )

    def _parse_gll(self, data: List[str]) -> bool:
        # GLL - Geographic Position - Latitude/Longitude

        if data is None or len(data) != 7:
            return False  # Unexpected number of params.
        data = _parse_data(_GLL, data)
        if data is None:
            return False  # Params didn't parse

        # Latitude
        self.latitude = _read_degrees(data, 0, "s")
        self.latitude_degrees, self.latitude_minutes = _read_int_degrees(data, 0, "s")

        # Longitude
        self.longitude = _read_degrees(data, 2, "w")
        self.longitude_degrees, self.longitude_minutes = _read_int_degrees(data, 2, "w")

        # UTC time of position
        self._update_timestamp_utc(data[4])

        # Status Valid(A) or Invalid(V)
        self.isactivedata = data[5]

        # Parse FAA mode indicator
        self._mode_indicator = data[6]

        return True

    def _parse_rmc(self, data: List[str]) -> bool:
        # RMC - Recommended Minimum Navigation Information

        if data is None or len(data) not in (12, 13):
            return False  # Unexpected number of params.
        data = _parse_data({12: _RMC, 13: _RMC_4_1}[len(data)], data)
        if data is None:
            self.fix_quality = 0
            return False  # Params didn't parse

        # UTC time of position and date
        self._update_timestamp_utc(data[0], data[8])

        # Status Valid(A) or Invalid(V)
        self.isactivedata = data[1]
        if data[1].lower() == "a":
            if self.fix_quality == 0:
                self.fix_quality = 1
        else:
            self.fix_quality = 0

        # Latitude
        self.latitude = _read_degrees(data, 2, "s")
        self.latitude_degrees, self.latitude_minutes = _read_int_degrees(data, 2, "s")

        # Longitude
        self.longitude = _read_degrees(data, 4, "w")
        self.longitude_degrees, self.longitude_minutes = _read_int_degrees(data, 4, "w")

        # Speed over ground, knots
        self.speed_knots = data[6]

        # Track made good, degrees true
        self.track_angle_deg = data[7]

        # Magnetic variation
        if data[9] is None or data[10] is None:
            self._magnetic_variation = None
        else:
            self._magnetic_variation = _read_degrees(data, 9, "w")

        # Parse FAA mode indicator
        self._mode_indicator = data[11]

        return True

    def _parse_gga(self, data: List[str]) -> bool:
        # GGA - Global Positioning System Fix Data

        if data is None or len(data) != 14:
            return False  # Unexpected number of params.
        data = _parse_data(_GGA, data)
        if data is None:
            self.fix_quality = 0
            return False  # Params didn't parse

        # UTC time of position
        self._update_timestamp_utc(data[0])

        # Latitude
        self.latitude = _read_degrees(data, 1, "s")
        self.latitude_degrees, self.latitude_minutes = _read_int_degrees(data, 1, "s")

        # Longitude
        self.longitude = _read_degrees(data, 3, "w")
        self.longitude_degrees, self.longitude_minutes = _read_int_degrees(data, 3, "w")

        # GPS quality indicator
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

    def _parse_gsa(self, talker: bytes, data: List[str]) -> bool:
        # GSA - GPS DOP and active satellites

        if data is None or len(data) not in (17, 18):
            return False  # Unexpected number of params.
        if len(data) == 17:
            data = _parse_data(_GSA, data)
        else:
            data = _parse_data(_GSA_4_11, data)
        if data is None:
            self.fix_quality_3d = 0
            return False  # Params didn't parse

        talker = str(talker, "ascii")

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

    def _parse_gsv(self, talker: bytes, data: List[str]) -> bool:
        # GSV - Satellites in view
        # pylint: disable=too-many-branches

        if data is None or len(data) not in (7, 11, 15, 19):
            return False  # Unexpected number of params.
        data = _parse_data(
            {7: _GSV7, 11: _GSV11, 15: _GSV15, 19: _GSV19}[len(data)],
            data,
        )
        if data is None:
            return False  # Params didn't parse

        talker = str(talker, "ascii")

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
                    for sat_id, sat_data in self.sats.items():
                        if (timestamp - sat_data[4]) > 30:
                            old.append(sat_id)
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
        self,
        i2c_bus: I2C,
        *,
        address: int = _GPSI2C_DEFAULT_ADDRESS,
        debug: bool = False,
        timeout: float = 5.0,
    ) -> None:
        from adafruit_bus_device import (  # pylint: disable=import-outside-toplevel
            i2c_device,
        )

        super().__init__(None, debug)  # init the parent with no UART
        self._i2c = i2c_device.I2CDevice(i2c_bus, address)
        self._lastbyte = None
        self._charbuff = bytearray(1)
        self._internalbuffer = []
        self._timeout = timeout

    def read(self, num_bytes: int = 1) -> bytearray:
        """Read up to num_bytes of data from the GPS directly, without parsing.
        Returns a bytearray with up to num_bytes or None if nothing was read"""
        result = []
        for _ in range(num_bytes):
            with self._i2c as i2c:
                # we read one byte at a time, verify it isnt part of a string of
                # 'stuffed' newlines and then append to our result array for byteification
                i2c.readinto(self._charbuff)
                char = self._charbuff[0]
                if (char == 0x0A) and (self._lastbyte != 0x0D):
                    continue  # skip duplicate \n's!
                result.append(char)
                self._lastbyte = char  # keep track of the last character approved
        return bytearray(result)

    def write(self, bytestr: ReadableBuffer) -> None:
        """Write a bytestring data to the GPS directly, without parsing
        or checksums"""
        with self._i2c as i2c:
            i2c.write(bytestr)

    @property
    def in_waiting(self) -> Literal[16]:
        """Returns number of bytes available in UART read buffer, always 16
        since I2C does not have the ability to know how much data is available"""
        return 16

    def readline(self) -> Optional[bytearray]:
        """Returns a newline terminated bytearray, must have timeout set for
        the underlying UART or this will block forever!"""
        timeout = time.monotonic() + self._timeout
        while timeout > time.monotonic():
            # check if our internal buffer has a '\n' termination already
            if self._internalbuffer and (self._internalbuffer[-1] == 0x0A):
                break
            char = self.read(1)
            if not char:
                continue
            self._internalbuffer.append(char[0])
            # print(bytearray(self._internalbuffer))
        if self._internalbuffer and self._internalbuffer[-1] == 0x0A:
            ret = bytearray(self._internalbuffer)
            self._internalbuffer = []  # reset the buffer to empty
            return ret
        return None  # no completed data yet
