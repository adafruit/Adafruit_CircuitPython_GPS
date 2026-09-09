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
    from typing import List, Optional, Tuple

    from busio import I2C, UART
    from circuitpython_typing import ReadableBuffer
    from typing_extensions import Literal
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
_VTG = 10
_ST_MIN = _GLL
_ST_MAX = _VTG

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
    # 10 - _VTG
    "fcFCfcfcC",
)


# Internal helper parsing functions.
# These handle input that might be none or null and return none instead of
# throwing errors.
def _split_decimal(nmea_data: str) -> Tuple[Optional[str], str]:
    # Split a numeric NMEA field into its integer and fractional digits.
    # Returns (None, "") if either part is missing or non-numeric. The
    # fractional part is optional; some receivers omit it entirely.
    dot = nmea_data.find(".")
    if dot == -1:
        int_part, minutes_decimal = nmea_data, "0"
    else:
        int_part, minutes_decimal = nmea_data[:dot], nmea_data[dot + 1 :]
        if not minutes_decimal:
            minutes_decimal = "0"
    if not int_part.isdigit() or not minutes_decimal.isdigit():
        return None, ""
    return int_part, minutes_decimal


def _parse_degrees(nmea_data: str) -> int:
    # Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
    # Where ddd is the degrees, mm.mmmm is the minutes.
    if nmea_data is None or len(nmea_data) < 3:
        return None
    # To avoid losing precision handle degrees and minutes separately
    # Return the final value as an integer. Further functions can parse
    # this into a float or separate parts to retain the precision
    int_part, minutes_decimal = _split_decimal(nmea_data)
    if int_part is None:
        return None
    degrees = int(int_part) // 100 * 1000000  # the ddd
    minutes = int(int_part) % 100  # the mm.
    minutes += int(f"{minutes_decimal[:4]:0<4}") / 10000
    minutes = int((minutes * 1000000) / 60)
    return degrees + minutes


def _parse_int(nmea_data: str) -> int:
    if nmea_data is None or not nmea_data:
        return None
    return int(nmea_data)


def _parse_float(nmea_data: str) -> float:
    if nmea_data is None or not nmea_data:
        return None
    return float(nmea_data)


def _parse_str(nmea_data: str) -> str:
    if nmea_data is None or not nmea_data:
        return None
    return str(nmea_data)


def _read_degrees(data: List[float], index: int, neg: str) -> float:
    # This function loses precision with float32
    if data[index] is None or data[index + 1] is None:
        return None
    x = data[index] / 1000000
    if data[index + 1].lower() == neg:
        x *= -1.0
    return x


def _read_deg_mins(data: List[str], index: int, neg: str) -> Tuple[int, float]:
    # the degrees come in different formats and vary between latitudes and
    # longitudes, which makes parsing tricky:
    # for latitudes: ddmm,mmmm (0 - 7 decimal places, not zero padded)
    # for longitudes: dddmm,mmmm (0 - 7 decimal places, not zero padded)
    if data[index] is None or len(data[index]) < 3:
        return None, None

    # we need to parse from right to left, minutes can only have 2 digits
    int_part, minutes_decimal = _split_decimal(data[index])
    if int_part is None or len(int_part) < 3:
        return None, None

    minutes_int = int_part[-2:]
    # the rest must be degrees which are either 2 or 3 digits
    deg = int(int_part[:-2])
    # combine the parts of the minutes, this also works when there are no
    # decimal places specified in the sentence
    minutes = float(f"{minutes_int}.{minutes_decimal}")
    if data[index + 1].lower() == neg:
        deg *= -1
    return deg, minutes


def _parse_talker(data_type: bytes) -> Tuple[bytes, bytes]:
    # Split the data_type into talker and sentence_type
    if data_type[:1] == b"P":  # Proprietary codes
        return (data_type[:1], data_type[1:])

    return (data_type[:2], data_type[2:])


def _parse_data(sentence_type: int, data: List[str]) -> Optional[List]:  # noqa: PLR0911
    # PLR0911: each early return is a distinct, self-documenting rejection reason ...
    """Parse sentence data for the specified sentence type and
    return a list of parameters in the correct format, or return None.
    """

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
                # A number parseable as degrees.
                # Required field so unparseable or empty should reject whole sentence.
                if nothing:
                    return None
                parsed_degrees = _parse_degrees(dti)
                if parsed_degrees is None:
                    return None
                params.append(parsed_degrees)
            elif pti == "D":
                # A number parseable as degrees or Nothing
                if nothing:
                    params.append(None)
                else:
                    params.append(_parse_degrees(dti))
            elif pti == "f":
                # A floating point number
                params.append(_parse_float(dti))
            elif pti == "F":
                # A floating point number or Nothing
                if nothing:
                    params.append(None)
                else:
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


class GPS:
    """GPS parsing module.  Can parse simple NMEA data sentences from serial
    GPS modules to read latitude, longitude, and more.
    """

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
        """The number of satellites in use (GGA), 0 - 12"""
        self.satellites_prev = None
        """The number of satellites in use from the previous data point, 0 - 12"""
        self.satellites_in_view = None
        """The number of satellites in view (GSV), 0 - 12"""
        self.satellites_in_view_prev = None
        """The number of satellites in view from the previous data point, 0 - 12"""

        self.horizontal_dilution = None
        """Horizontal dilution of precision (GGA)"""
        self.altitude_m = None
        """Antenna altitude relative to mean sea level"""
        self.height_geoid = None
        """Geoidal separation relative to WGS 84"""
        self.speed_knots = None
        """Ground speed in knots"""
        self.speed_kmh = None
        """Ground speed in km/h"""
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
        if talker not in {b"GA", b"GB", b"GI", b"GL", b"GP", b"GQ", b"GN"}:
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
        elif sentence_type == b"VTG":  # Ground speed
            result = self._parse_vtg(args)

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
            self.write(bytes(f"{checksum:02x}".upper(), "ascii"))
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

    @staticmethod
    def _parse_hhmmss(time_utc: Optional[str]) -> Optional[Tuple[int, int, int]]:
        # Parse an NMEA hhmmss(.sss) field into (hours, mins, secs).
        # Returns None if the field is absent, short, or non-numeric.
        if time_utc is None or len(time_utc) < 6 or not time_utc[0:6].isdigit():
            return None
        return (int(time_utc[0:2]), int(time_utc[2:4]), int(time_utc[4:6]))

    @staticmethod
    def _parse_ddmmyy(date: Optional[str]) -> Optional[Tuple[int, int, int]]:
        # Parse an NMEA ddmmyy field into (day, month, year).
        # Returns None if the field is absent, short, or non-numeric.
        if date is None or len(date) < 6 or not date[0:6].isdigit():
            return None
        return (int(date[0:2]), int(date[2:4]), 2000 + int(date[4:6]))

    def _update_timestamp_utc(self, time_utc: str, date: Optional[str] = None) -> None:
        parsed_time = self._parse_hhmmss(time_utc)
        if parsed_time is None:
            # Malformed time field; leave the previous timestamp untouched.
            return
        hours, mins, secs = parsed_time

        parsed_date = self._parse_ddmmyy(date)
        if parsed_date is None:
            # No usable date field; carry forward the date we already have.
            if self.timestamp_utc is None:
                day, month, year = 0, 0, 0
            else:
                day = self.timestamp_utc.tm_mday
                month = self.timestamp_utc.tm_mon
                year = self.timestamp_utc.tm_year
        else:
            day, month, year = parsed_date

        self.timestamp_utc = time.struct_time((year, month, day, hours, mins, secs, 0, 0, -1))

    def _parse_vtg(self, data: List[str]) -> bool:
        # VTG - Course Over Ground and Ground Speed

        if data is None or len(data) != 9:
            return False  # Unexpected number of params

        parsed_data = _parse_data(_VTG, data)
        if parsed_data is None:
            return False  # Params didn't parse

        # Track made good, degrees true
        self.track_angle_deg = parsed_data[0]

        # Speed over ground, knots
        self.speed_knots = parsed_data[4]

        # Speed over ground, kilometers / hour
        self.speed_kmh = parsed_data[6]

        # Parse FAA mode indicator
        self._mode_indicator = parsed_data[8]

        return True

    def _parse_gll(self, data: List[str]) -> bool:
        # GLL - Geographic Position - Latitude/Longitude

        if data is None or len(data) != 7:
            return False  # Unexpected number of params.
        parsed_data = _parse_data(_GLL, data)
        if parsed_data is None:
            return False  # Params didn't parse

        # Latitude
        self.latitude = _read_degrees(parsed_data, 0, "s")
        self.latitude_degrees, self.latitude_minutes = _read_deg_mins(data=data, index=0, neg="s")

        # Longitude
        self.longitude = _read_degrees(parsed_data, 2, "w")
        self.longitude_degrees, self.longitude_minutes = _read_deg_mins(data=data, index=2, neg="w")

        # UTC time of position
        self._update_timestamp_utc(parsed_data[4])

        # Status Valid(A) or Invalid(V)
        self.isactivedata = parsed_data[5]

        # Parse FAA mode indicator
        self._mode_indicator = parsed_data[6]

        return True

    def _parse_rmc(self, data: List[str]) -> bool:
        # RMC - Recommended Minimum Navigation Information

        if data is None or len(data) not in {12, 13}:
            return False  # Unexpected number of params.
        parsed_data = _parse_data({12: _RMC, 13: _RMC_4_1}[len(data)], data)
        if parsed_data is None:
            self.fix_quality = 0
            return False  # Params didn't parse

        # UTC time of position and date
        self._update_timestamp_utc(parsed_data[0], parsed_data[8])

        # Status Valid(A) or Invalid(V)
        self.isactivedata = parsed_data[1]
        if parsed_data[1].lower() == "a":
            if self.fix_quality == 0:
                self.fix_quality = 1
        else:
            self.fix_quality = 0

        # Latitude
        self.latitude = _read_degrees(parsed_data, 2, "s")
        self.latitude_degrees, self.latitude_minutes = _read_deg_mins(data=data, index=2, neg="s")

        # Longitude
        self.longitude = _read_degrees(parsed_data, 4, "w")
        self.longitude_degrees, self.longitude_minutes = _read_deg_mins(data=data, index=4, neg="w")

        # Speed over ground, knots
        self.speed_knots = parsed_data[6]

        # Track made good, degrees true
        self.track_angle_deg = parsed_data[7]

        # Magnetic variation
        if parsed_data[9] is None or parsed_data[10] is None:
            self._magnetic_variation = None
        else:
            self._magnetic_variation = _read_degrees(parsed_data, 9, "w")

        # Parse FAA mode indicator
        self._mode_indicator = parsed_data[11]

        return True

    def _parse_gga(self, data: List[str]) -> bool:
        # GGA - Global Positioning System Fix Data

        if data is None or len(data) != 14:
            return False  # Unexpected number of params.
        parsed_data = _parse_data(_GGA, data)
        if parsed_data is None:
            self.fix_quality = 0
            return False  # Params didn't parse

        # UTC time of position
        self._update_timestamp_utc(parsed_data[0])

        # Latitude
        self.latitude = _read_degrees(parsed_data, 1, "s")
        self.longitude_degrees, self.longitude_minutes = _read_deg_mins(data=data, index=3, neg="w")

        # Longitude
        self.longitude = _read_degrees(parsed_data, 3, "w")
        self.latitude_degrees, self.latitude_minutes = _read_deg_mins(data=data, index=1, neg="s")

        # GPS quality indicator
        self.fix_quality = parsed_data[5]

        # Capture previous value
        self.satellites_prev = self.satellites

        # Number of satellites in use, 0 - 12
        self.satellites = parsed_data[6]

        # Horizontal dilution of precision
        self.horizontal_dilution = parsed_data[7]

        # Antenna altitude relative to mean sea level
        self.altitude_m = _parse_float(parsed_data[8])
        # data[9] - antenna altitude unit, always 'M' ???

        # Geoidal separation relative to WGS 84
        self.height_geoid = _parse_float(parsed_data[10])
        # data[11] - geoidal separation unit, always 'M' ???

        # data[12] - Age of differential GPS data, can be null
        # data[13] - Differential reference station ID, can be null

        return True

    def _parse_gsa(self, talker: bytes, data: List[str]) -> bool:
        # GSA - GPS DOP and active satellites

        if data is None or len(data) not in {17, 18}:
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
            self.sat_prns.append(f"{talker}{sat}")

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

        if data is None or len(data) not in {7, 11, 15, 19}:
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

        # Capture previous value
        self.satellites_in_view_prev = self.satellites_in_view

        # Number of satellites in view
        self.satellites_in_view = data[2]

        sat_tup = data[3:]

        satlist = []
        timestamp = time.monotonic()
        for i in range(len(sat_tup) // 4):
            j = i * 4
            value = (
                # Satellite number
                f"{talker}{sat_tup[0 + j]}",
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
            if len(self._sats) == self.satellites_in_view:
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

        return True


_GPSI2C_MAX_PENDING = const(128)  # NMEA sentences are <= 82 bytes; an unterminated
# buffer past this means a terminator was lost, so
# resync rather than growing without bound (issue #123).
_GPSI2C_MIN_PACKET = const(11)  # smallest viable sentence; matches the gate in
# _read_sentence() before it attempts a readline().


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
        chunk_size: int = 32,
    ) -> None:
        from adafruit_bus_device import i2c_device  # noqa: PLC0415

        super().__init__(None, debug)  # init the parent with no UART
        self._i2c = i2c_device.I2CDevice(i2c_bus, address)
        self._lastbyte = None
        self._chunk = bytearray(chunk_size)
        self._pending = bytearray()
        self._timeout = timeout

    def _fill(self) -> int:
        """Read one chunk from the module into the pending buffer.

        The GTop I2C protocol has no way to report how much data is waiting: the
        module returns 0x0A filler bytes when it has nothing to send. Those are
        discarded here, so a return value of 0 means the module is idle and there
        is no point asking again until more time has passed.

        Returns the number of real bytes recovered.
        """
        with self._i2c as i2c:
            i2c.readinto(self._chunk)
        recovered = 0
        for char in self._chunk:
            if char == 0x0A and self._lastbyte != 0x0D:
                continue  # filler LF (a real LF only ever follows a CR)
            self._pending.append(char)
            self._lastbyte = char  # last approved byte, for CR/LF across chunks
            recovered += 1
        return recovered

    def _newline_index(self) -> int:
        """Index of the first 0x0A in the pending buffer, or -1.

        Written as an explicit scan rather than bytearray.find() so it does not
        depend on which string methods a given CircuitPython build provides.
        """
        for index, char in enumerate(self._pending):
            if char == 0x0A:
                return index
        return -1

    def _resync(self) -> None:
        """Discard the pending buffer after a lost terminator so it cannot grow
        without bound.  Clearing ``_lastbyte`` prevents a stale CR from wrongly
        preserving the next filler LF."""
        self._pending = bytearray()

        self._lastbyte = None

    def read(self, num_bytes: int = 1) -> bytearray:
        """Read up to num_bytes of data from the GPS directly, without parsing.
        Returns a bytearray with up to num_bytes, which may be empty if the
        module had nothing to send or the timeout expired."""
        deadline = time.monotonic() + self._timeout
        while len(self._pending) < num_bytes:
            if self._fill() == 0:
                break  # module is idle; return what we have
            if time.monotonic() > deadline:
                break
        result = self._pending[:num_bytes]
        self._pending[:num_bytes] = b""  # in-place shrink
        return result

    def write(self, bytestr: ReadableBuffer) -> None:
        """Write a bytestring data to the GPS directly, without parsing
        or checksums"""
        with self._i2c as i2c:
            i2c.write(bytestr)

    @property
    def in_waiting(self) -> int:
        """Number of recovered bytes not yet consumed.

        I2C cannot report a UART-style buffer level, so this pulls a chunk when
        the pending buffer is below the threshold ``_read_sentence()`` needs
        before it will attempt a ``readline()``.
        """
        while len(self._pending) < _GPSI2C_MIN_PACKET:
            if self._fill() == 0:
                break  # module idle; report what we actually have
        return len(self._pending)

    def readline(self) -> Optional[bytearray]:
        """Return a newline-terminated bytearray, or None if no complete
        sentence is available yet.

        Returns as soon as the module reports idle rather than spinning until
        the timeout expires.  A partial sentence is retained and completed on a
        later call; a partial that grows past a legal NMEA length is discarded
        as a lost terminator.
        """
        deadline = time.monotonic() + self._timeout
        while True:
            newline = self._newline_index()
            if newline >= 0:
                line = self._pending[: newline + 1]
                self._pending[: newline + 1] = b""  # in-place shrink
                return line
            if len(self._pending) > _GPSI2C_MAX_PENDING:
                self._resync()  # unterminated and over-long => garbage
            if self._fill() == 0:
                return None  # module idle; no completed sentence yet
            if time.monotonic() > deadline:
                return None
