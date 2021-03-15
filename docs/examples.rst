Simple test
------------

Ensure your device works with this simple test.

.. literalinclude:: ../examples/gps_simpletest.py
    :caption: examples/gps_simpletest.py
    :linenos:

Echo test
---------

Simple GPS module demonstration. This will print NMEA sentences received from
the GPS, great for testing connection. This uses the GPS to send some
commands, then reads directly from the GPS.

.. literalinclude:: ../examples/gps_echotest.py
    :caption: examples/gps_echotest.py
    :linenos:

Time source
-----------

Simple script using GPS timestamps as RTC time source. The GPS timestamps are
available without a full location fix if a single satellite can be seen. The
GPS unit will keep the track of time while there is power source (i.e. a coin
cell battery.)

.. literalinclude:: ../examples/gps_time_source.py
    :caption: examples/gps_time_source.py
    :linenos:

Data logging
------------

Simple GPS datalogging demonstration. This example uses the GPS library and to
read raw NMEA sentences over I2C or UART from the GPS unit and dumps them to a
file on an SD card (recommended), microcontroller internal storage (be careful
as only a few kilobytes are available), or to a filesystem.

If you are using a microcontroller, before writing to internal storage you
MUST carefully follow the steps in this guide to enable writes to the internal
filesystem:
`Writing to the filesystem
<https://learn.adafruit.com/cpu-temperature-logging-with-circuit-python/writing-to-the-filesystem>`_

.. literalinclude:: ../examples/gps_datalogging.py
    :caption: examples/gps_datalogging.py
    :linenos:

Satellite fix
-------------

This example uses GSA and GSV sentences from the GPS device to report on the
quality of the received data from the satellites.

* GSA - DOP(Dilution of Precision) and active satellites
* GSV - Satellites in view

.. literalinclude:: ../examples/gps_satellitefix.py
    :caption: examples/gps_satellitefix.py
    :linenos:
