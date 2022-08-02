Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-gps/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/gps/en/latest/
    :alt: Documentation Status

.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_GPS/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_GPS/actions/
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

GPS parsing module.  Can send commands to, and parse simple NMEA data sentences
from serial and I2C GPS modules to read latitude, longitude, and more.


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installing from PyPI
====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-gps/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-gps

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-gps

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .venv/bin/activate
    pip3 install adafruit-circuitpython-gps

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


Note: Sending multiple PMTK314 packets with ``gps.send_command()`` will not
work unless there is a substantial amount of time in-between each time
``gps.send_command()`` is called. A ``time.sleep()`` of 1 second or more
should fix this.

Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/gps/en/latest/>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_gps/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
