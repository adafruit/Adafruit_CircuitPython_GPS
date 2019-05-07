
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

This isn't ideal for gps data as this lowers the accuracty from 0.1m to 11m. 

This can be fixed by using string formatting when the gps data is outputted.

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

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_gps/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

To build this library locally you'll need to install the
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
