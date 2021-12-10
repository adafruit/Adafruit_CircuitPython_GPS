# pylint: disable=missing-function-docstring,missing-module-docstring,invalid-name,protected-access,no-self-use,missing-class-docstring

# SPDX-FileCopyrightText: 2021 Jonas Kittner
#
# SPDX-License-Identifier: MIT
import time
from unittest import mock
from freezegun import freeze_time
import pytest

from adafruit_gps import _parse_degrees
from adafruit_gps import _parse_int
from adafruit_gps import _parse_float
from adafruit_gps import _parse_str
from adafruit_gps import _read_degrees
from adafruit_gps import _parse_talker
from adafruit_gps import _parse_data
from adafruit_gps import GPS


@pytest.mark.parametrize(
    ("val", "exp"),
    (
        pytest.param("0023.456", 0.390933, id="leading zero"),
        pytest.param("6413.9369", 64.23228, id="regular value"),
        pytest.param("2747.416122087989", 27.79027, id="long value"),
    ),
)
def test_parse_degrees(val, exp):
    assert _parse_degrees(val) == pytest.approx(exp)


def test_parse_degrees_too_short():
    assert _parse_degrees("12") is None


def test_parse_int():
    assert _parse_int("456") == 456


@pytest.mark.parametrize(
    "val",
    (None, ""),
)
def test_parse_int_invalid(val):
    assert _parse_int(val) is None


def test_parse_float():
    assert _parse_float("456") == 456


@pytest.mark.parametrize(
    "val",
    (None, ""),
)
def test_parse_float_invalid(val):
    assert _parse_float(val) is None


@pytest.mark.parametrize(
    ("data", "neg", "exp"),
    (
        pytest.param([27.79027, "S"], "s", -27.79027, id="south negative"),
        pytest.param([64.23228, "N"], "s", 64.23228, id="north not negative"),
        pytest.param([123.4567, "W"], "w", -123.4567, id="west negative"),
        pytest.param([10.7891, "E"], "w", 10.7891, id="east not negative"),
    ),
)
def test_read_degrees(data, neg, exp):
    assert _read_degrees(data, 0, neg) == exp


@pytest.mark.parametrize(
    "val",
    (None, ""),
)
def test_parse_str_invalid(val):
    assert _parse_str(val) is None


def test_parse_str_valid():
    assert _parse_str(13) == "13"


def test_parse_talker_prop_code():
    assert _parse_talker(b"PMTK001") == (b"P", b"MTK001")


def test_parse_talker_regular():
    assert _parse_talker(b"GPRMC") == (b"GP", b"RMC")


@pytest.mark.parametrize(
    "sentence_type",
    (-1, 10),
)
def test_parse_data_unknown_sentence_type(sentence_type):
    assert _parse_data(sentence_type, data=[]) is None


def test_param_types_does_not_match_data_items():
    assert _parse_data(sentence_type=1, data=["too", "few", "items"]) is None


def test_parse_data_unexpected_parameter_type():
    with mock.patch("adafruit_gps._SENTENCE_PARAMS", ("xyz",)):
        with pytest.raises(TypeError) as exc_info:
            _parse_data(sentence_type=0, data=["a", "b", "c"])

    assert exc_info.value.args[0] == "GPS: Unexpected parameter type 'x'"


class UartMock:
    """mocking the UART connection an its methods"""

    def write(self, bytestr):
        print(bytestr, end="")

    @property
    def in_waiting(self):
        return 100


def test_read_sentence_too_few_in_waiting():
    with mock.patch.object(GPS, "readline", return_value="x"):

        class UartMockWaiting(UartMock):
            @property
            def in_waiting(self):
                # overwrite the in_waiting property to perform the test
                return 3

        gps = GPS(uart=UartMockWaiting())
        assert not gps.update()


def test_GPS_update_timestamp_UTC_date_None():
    gps = GPS(uart=UartMock())
    assert gps.datetime is None
    assert gps.timestamp_utc is None
    exp_struct = time.struct_time((0, 0, 0, 22, 14, 11, 0, 0, -1))
    gps._update_timestamp_utc(time_utc="221411")
    assert gps.timestamp_utc == exp_struct


def test_GPS_update_timestamp_UTC_date_not_None():
    gps = GPS(uart=UartMock())
    exp_struct = time.struct_time((2021, 10, 2, 22, 14, 11, 0, 0, -1))
    gps._update_timestamp_utc(time_utc="221411", date="021021")
    assert gps.timestamp_utc == exp_struct


def test_GPS_update_timestamp_timestamp_utc_was_not_none_new_date_none():
    gps = GPS(uart=UartMock())
    # set this to a value
    gps.timestamp_utc = time.struct_time((2021, 10, 2, 22, 10, 11, 0, 0, -1))
    exp_struct = time.struct_time((2021, 10, 2, 22, 14, 11, 0, 0, -1))
    # update the timestamp
    gps._update_timestamp_utc(time_utc="221411")
    assert gps.timestamp_utc == exp_struct


def test_GPS_update_with_unknown_talker():
    r = b"$XYRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*7c\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()


def test_GPS_update_rmc_no_magnetic_variation():
    r = b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*6A\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        exp_time = time.struct_time((2021, 10, 2, 21, 50, 32, 0, 0, -1))
        assert gps.timestamp_utc == exp_time
        assert gps.latitude == pytest.approx(12.57613)
        assert gps.longitude == pytest.approx(1.385391)
        assert gps.fix_quality == 1
        assert gps.fix_quality_3d == 0
        assert gps.speed_knots == 0.45
        assert gps.track_angle_deg == 56.35
        assert gps._magnetic_variation is None
        assert gps._mode_indicator == "A"
        assert gps.has_fix is True
        assert gps.has_3d_fix is False
        assert gps.datetime == exp_time
        assert (
            gps._raw_sentence
            == "$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*6A"
        )
        assert (
            gps.nmea_sentence
            == "$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*6A"
        )


def test_GPS_update_rmc_fix_is_set():
    r_valid = (
        b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*6A\r\n"
    )
    r_invalid = (
        b"$GPRMC,215032.086,V,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*7D\r\n"
    )
    with mock.patch.object(GPS, "readline", return_value=r_valid):
        gps = GPS(uart=UartMock())
        assert gps.update()
        assert gps.fix_quality == 1
        assert gps.has_fix is True

    with mock.patch.object(gps, "readline", return_value=r_invalid):
        assert gps.update()
        assert gps.fix_quality == 0
        assert gps.has_fix is False


def test_GPS_update_rmc_fix_is_set_new():
    r_valid = (
        b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*6A\r\n"
    )
    r_invalid = b"$GPRMC,215032.086,V,ABC,N,00123.12345,E,0.45,56.35,021021,,,A*1B\r\n"
    with mock.patch.object(GPS, "readline", return_value=r_valid):
        gps = GPS(uart=UartMock())
        assert gps.update()
        assert gps.fix_quality == 1
        assert gps.has_fix is True
    # now get an invalid response --> set fix_quality to 0
    with mock.patch.object(gps, "readline", return_value=r_invalid):
        assert not gps.update()
        assert gps.fix_quality == 0
        assert gps.has_fix is False


def test_GPS_update_rmc_invalid_checksum():
    r = b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*5C\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert not gps.update()


def test_GPS_update_empty_sentence():
    with mock.patch.object(GPS, "readline", return_value=b""):
        gps = GPS(uart=UartMock())
        assert not gps.update()


@pytest.mark.parametrize(
    ("r", "exp"),
    (
        pytest.param(
            b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,1234.56,W,A*14\r\n",
            -12.576,
            id="W",
        ),
        pytest.param(
            b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,1234.56,E,A*06\r\n",
            12.576,
            id="E",
        ),
    ),
)
def test_GPS_update_rmc_has_magnetic_variation(r, exp):
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        assert gps._magnetic_variation == pytest.approx(exp)


def test_parse_sentence_invalid_delimiter():
    with mock.patch.object(GPS, "readline", return_value=b"a;b;c;d;12*66"):
        gps = GPS(uart=UartMock())
        assert gps._parse_sentence() is None


def test_GPS_update_sentence_is_None():
    with mock.patch.object(GPS, "_parse_sentence", return_value=None):
        gps = GPS(uart=UartMock())
        assert not gps.update()


def test_GPS_update_rmc_debug_shows_sentence(capsys):
    r = b"$GPRMC,215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A*6A\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock(), debug=True)
        assert gps.update()
        out, err = capsys.readouterr()
        assert err == ""
        assert (
            out
            == "('GPRMC', '215032.086,A,1234.5678,N,00123.12345,E,0.45,56.35,021021,,,A')\n"
        )


def test_GPS_update_data_type_too_short():
    r = ("GPRM", "x,y,z")
    with mock.patch.object(GPS, "_parse_sentence", return_value=r):
        gps = GPS(uart=UartMock(), debug=True)
        assert not gps.update()


def test_GPS_send_command_with_checksum(capsys):
    gps = GPS(uart=UartMock())
    gps.send_command(command=b"$PMTK001,314,3\r\n", add_checksum=True)
    out, err = capsys.readouterr()
    assert err == ""
    assert out == ("b'$'" "b'$PMTK001,314,3\\r\\n'" "b'*'" "b'15'" "b'\\r\\n'")


def test_GPS_send_command_without_checksum(capsys):
    gps = GPS(uart=UartMock())
    gps.send_command(command=b"$PMTK001,314,3\r\n", add_checksum=False)
    out, err = capsys.readouterr()
    assert err == ""
    assert out == ("b'$'" "b'$PMTK001,314,3\\r\\n'" "b'\\r\\n'")


def test_GPS_update_from_GLL():
    r = b"$GPGLL,4916.45,N,12311.12,W,225444,A,A*5c\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        exp_time = time.struct_time((0, 0, 0, 22, 54, 44, 0, 0, -1))
        assert gps.timestamp_utc == exp_time
        assert gps.latitude == pytest.approx(49.27417)
        assert gps.longitude == pytest.approx(-123.1853)
        assert gps.isactivedata == "A"
        assert gps._mode_indicator == "A"
        assert gps.fix_quality == 0
        assert gps.fix_quality_3d == 0
        assert gps.has_fix is False
        assert gps.has_3d_fix is False
        assert gps._raw_sentence == "$GPGLL,4916.45,N,12311.12,W,225444,A,A*5c"
        assert gps.nmea_sentence == "$GPGLL,4916.45,N,12311.12,W,225444,A,A*5c"


def test_GPS_update_from_RMC():
    r = b"$GNRMC,001031.00,A,4404.13993,N,12118.86023,W,0.146,084.4,100117,,,A*5d\r\n"
    # TODO: length 13 and 14 version
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        exp_time = time.struct_time((2017, 1, 10, 0, 10, 31, 0, 0, -1))
        assert gps.timestamp_utc == exp_time
        assert gps.datetime == exp_time
        assert gps.isactivedata == "A"
        assert gps.fix_quality == 1
        assert gps.has_fix is True
        assert gps.has_3d_fix is False
        assert gps.latitude == pytest.approx(44.069)
        assert gps.longitude == pytest.approx(-121.3143)
        assert gps.speed_knots == 0.146
        assert gps.track_angle_deg == 84.4
        assert gps._magnetic_variation is None
        assert gps._mode_indicator == "A"


def test_GPS_update_from_GGA():
    r = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        exp_time = time.struct_time((0, 0, 0, 12, 35, 19, 0, 0, -1))
        assert gps.timestamp_utc == exp_time
        assert gps.latitude == pytest.approx(48.1173)
        assert gps.longitude == pytest.approx(11.51667)
        assert gps.fix_quality == 1
        assert gps.fix_quality_3d == 0
        assert gps.satellites == 8
        assert gps.horizontal_dilution == 0.9
        assert gps.altitude_m == 545.4
        assert gps.height_geoid == 46.9
        assert gps.has_fix is True
        assert gps.has_3d_fix is False
        assert gps.datetime == exp_time
        assert (
            gps._raw_sentence
            == "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        )
        assert (
            gps.nmea_sentence
            == "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        )


@pytest.mark.parametrize(
    "r",
    (
        pytest.param(
            b"$GPGSA,A,3,15,18,14,,,31,,,23,,,,04.5,02.1,04.0*0f\r\n", id="smaller v4.1"
        ),
        pytest.param(
            b"$GPGSA,A,3,15,18,14,,,31,,,23,,,,04.5,02.1,04.0,3*10\r\n",
            id="greater v4.1",
        ),
    ),
)
def test_GPS_update_from_GSA(r):
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        assert gps.sel_mode == "A"
        assert gps.fix_quality_3d == 3
        # assert gps.has_fix is True  # TODO: shouldn't this be True?
        assert gps.has_3d_fix is True
        assert gps.sat_prns == ["GP15", "GP18", "GP14", "GP31", "GP23"]
        assert gps.pdop == 4.5
        assert gps.hdop == 2.1
        assert gps.vdop == 4.0


def test_GPS_update_from_GSV_first_part():
    r = b"$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75\r\n"
    with mock.patch.object(GPS, "readline", return_value=r):
        gps = GPS(uart=UartMock())
        assert gps.update()
        assert gps.total_mess_num == 2
        assert gps.mess_num == 1
        assert gps.satellites == 8
        # check two satellites, without timestamp, since it is dynamic
        sats = gps._sats
        assert sats[0][:-1] == ("GP1", 40, 83, 46)
        assert sats[-1][:-1] == ("GP14", 22, 228, 45)

        # check at least that timestamp is there
        assert isinstance(sats[0][4], float)
        assert isinstance(sats[-1][4], float)
        assert (
            gps._raw_sentence
            == "$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75"
        )
        assert (
            gps.nmea_sentence
            == "$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75"
        )


def test_GPS_update_from_GSV_both_parts_sats_are_removed():
    gps = GPS(uart=UartMock())
    with mock.patch.object(GPS, "readline") as m:
        with freeze_time("2021-10-20 19:00:00"):
            # first part of the request
            m.return_value = b"$GPGSV,2,1,04,01,40,083,46,02,17,308,41*78\r\n"
            assert gps.update()
            assert gps.total_mess_num == 2
            assert gps.mess_num == 1
            assert gps.satellites == 4
            # first time we received satellites, so this must be None
            assert gps.sats is None
        # some time has passed so the first two satellites will be too old, but
        # this one not
        with freeze_time("2021-10-20 19:00:20"):
            # second part of the request
            m.return_value = b"$GPGSV,2,2,04,12,07,344,39,14,22,228,45*7c\r\n"
            assert gps.update()
            assert gps.total_mess_num == 2
            assert gps.mess_num == 2
            assert gps.satellites == 4
            # we should now have 4 satellites from the two part request
            assert set(gps.sats.keys()) == {"GP1", "GP2", "GP12", "GP14"}

        # time passed (more than 30 seconds) and the next request does not
        # contain the previously seen satellites but two new ones
        with freeze_time("2021-10-20 19:00:31"):
            # a third, one part request
            m.return_value = b"$GPGSV,1,1,02,13,07,344,39,15,22,228,45*7a\r\n"
            assert gps.update()
            assert gps.satellites == 2
            assert set(gps.sats.keys()) == {"GP12", "GP14", "GP13", "GP15"}
