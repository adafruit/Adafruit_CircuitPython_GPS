
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-gps/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/gps/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.com/adafruit/Adafruit_CircuitPython_GPS.svg?branch=master
    :target: https://travis-ci.com/adafruit/Adafruit_CircuitPython_GPS
    :alt: Build Status

GPS parsing module.  Can parse simple NMEA data sentences from serial GPS
modules to read latitude, longitude, and more.


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============

See examples/gps_simpletest.py for a demonstration of parsing and printing GPS location.

Important: 
Feather boards and many other circuitpython boards will round to two decimal places like this:

.. code-block:: python

    >>> float('1234.5678')
    1234.57

This isn't ideal for GPS data as this lowers the accuracy from 0.1m to 11m. 

This can be fixed by using string formatting when the GPS data is output.

An implementation of this can be found in examples/gps_simpletest.py

.. code-block:: python

    import time
    import board
    import busio

    import adafruit_gps

    RX = board.RX
    TX = board.TX

    uart = busio.UART(TX, RX, baudrate=9600, timeout=30)

    gps = adafruit_gps.GPS(uart, debug=False)

    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

    gps.send_command(b'PMTK220,1000')

    last_print = time.monotonic()
    while True:

        gps.update()

        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
            if not gps.has_fix:
                print('Waiting for fix...')
                continue
            print('=' * 40)  # Print a separator line.
            print('Latitude: {0:.6f} degrees'.format(gps.latitude))
            print('Longitude: {0:.6f} degrees'.format(gps.longitude))


These two lines are the lines that actually solve the issue:

.. code-block:: python

    print('Latitude: {0:.6f} degrees'.format(gps.latitude))
    print('Longitude: {0:.6f} degrees'.format(gps.longitude))
   
About NMEA Data
===============
This GPS module uses the NMEA 0183 protocol.

This data is formatted by the GPS in one of two ways.

The first of these is GGA. GGA has more or less everything you need.

Here's an explanation of GGA:
::

                                                        11
           1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
           |         |       | |        | | |  |   |   | |   | |   |    |
    $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh


1. Time (UTC)
2. Latitude
3. N or S (North or South)
4. Longitude
5. E or W (East or West)
6. GPS Quality Indicator,

   * 0 - fix not available,
   * 1 - GPS fix,
   * 2 - Differential GPS fix
    
7. Number of satellites in view, 00 - 12
8. Horizontal Dilution of precision
9. Antenna Altitude above/below mean-sea-level (geoid)
10. Units of antenna altitude, meters
11. Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
12. Units of geoidal separation, meters
13. Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
14. Differential reference station ID, 0000-1023
15. Checksum

The second of these is RMC. RMC is Recommended Minimum Navigation Information.

Here's an explanation of RMC:
::

                                                               12
           1         2 3       4 5        6 7   8   9   10   11|
           |         | |       | |        | |   |   |    |   | |
    $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh

1. Time (UTC)
2. Status, V = Navigation receiver warning
3. Latitude
4. N or S
5. Longitude
6. E or W
7. Speed over ground, knots
8. Track made good, degrees true
9. Date, ddmmyy
10. Magnetic Variation, degrees
11. E or W
12. Checksum


`Info about NMEA taken from here
<https://www.tronico.fi/OH6NT/docs/NMEA0183.pdf>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_gps/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

To build this library locally, you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-gps --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
