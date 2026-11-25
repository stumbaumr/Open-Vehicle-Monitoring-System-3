===========================
Bluetooth ELM327 Emulation
===========================

The ELM327 emulation lets the OVMS module act as a Bluetooth Low Energy (BLE) OBD adapter.
Apps that support BLE ELM327 adapters (for example route planners and OBD monitoring apps)
can connect to the module and send their OBD2/UDS requests. The module forwards each request
to the vehicle CAN bus with the OVMS poller, and returns the ECU response in the ELM327 format.

.. warning::

  **This is a work in progress.** The emulation has not yet been verified with all apps.
  App compatibility depends on the BLE service UUIDs, the device name and the ELM327 commands
  that the app uses. Please report your results on the developer mailing list.

  The emulation gives the connected app **direct access to the vehicle CAN bus**. Only
  allow the services that the app needs (see ``elm327.services``).


-------------
Build options
-------------

Bluetooth is not included in the standard firmware builds. In a Bluetooth build, the
ELM327 emulation is the only Bluetooth service. To build a firmware with Bluetooth support,
use the Bluetooth configuration template::

  cp support/sdkconfig.bluetooth.hw31 sdkconfig
  make

The template is the standard hardware 3.1 configuration with these changes:

- Bluetooth controller in BLE only mode, Bluedroid host with GATT server and SMP, no GATT client
- Bluetooth memory allocation from SPI RAM first, dynamic environment memory, 2 connections
- ``CONFIG_ESP32_WIFI_IRAM_OPT`` and ``CONFIG_ESP32_WIFI_RX_IRAM_OPT`` disabled to free IRAM
  for the Bluetooth controller (WiFi throughput is slightly lower)
- 8 instead of 16 WiFi static RX buffers, no WiFi AMPDU RX, no WiFi CSI (channel state
  information, not used by OVMS), 1 BLE connection, no BLE scan duplicate filter: this reduces
  the internal RAM usage

To build with the NimBLE host stack instead of Bluedroid, use the NimBLE template::

  cp support/sdkconfig.bluetooth-nimble.hw31 sdkconfig
  make

NimBLE needs less internal RAM and approximately 230 KB less flash than Bluedroid. The template
puts the NimBLE buffers in SPI RAM and only enables the peripheral role. The function and the
commands are the same with both host stacks.

.. warning::

  The Bluetooth controller and stack need approximately 40 KB or more of the internal RAM of the
  ESP32. With a cellular modem, the vehicle module and the server connections, the free internal
  RAM can become too low. Check the free internal RAM with ``module memory`` (``Free 8-bit``).


-----
Setup
-----

1. Make sure that a vehicle module is loaded and that the CAN bus is in active mode. The
   vehicle module normally starts the bus. With the empty vehicle module ``NONE``, start the
   bus manually, for example: ``can can1 start active 500000``.
2. Bluetooth starts automatically 60 seconds after the boot (``[auto] bluetooth``, default
   ``yes``). The delay lets the network, modem and server connections allocate their memory
   first. If less than ``[bluetooth] minfree`` bytes of internal RAM are free, the automatic
   start is skipped and the log shows an error. To start or stop Bluetooth manually::

     power bluetooth on
     power bluetooth off

3. Open the pairing window, then connect from the app within 2 minutes::

     bt elm327 pair

   The phone shows a pairing request. Accept it. The module stores the bond, so the phone
   reconnects later without a new pairing window.
4. Check the status::

     bt elm327 status

The module advertises the service UUID ``FFF0`` with the configured device name, and uses
its public Bluetooth address.

**Security:** all characteristics require an encrypted connection. A new device can only
bond ("Just Works" pairing) while the pairing window is open. Bonded devices reconnect at any
time. To remove a bond, use ``bt clear <mac>``.


--------------
Configuration
--------------

=================================== ================ =================================================
Config                              Default          Description
=================================== ================ =================================================
[auto] bluetooth                    yes              Start Bluetooth at boot
[bluetooth] minfree                 40000            Automatic start: minimum free internal RAM
                                                     in bytes
[bluetooth] elm327.name             OBDII            BLE device name (restart Bluetooth to apply)
[bluetooth] elm327.bus              can1             CAN bus for the requests
[bluetooth] elm327.protocol         6                Protocol for ``ATSP0`` (auto): 6 = CAN 11/500,
                                                     7 = CAN 29/500, 8 = CAN 11/250, 9 = CAN 29/250
[bluetooth] elm327.timeout          500              Minimum response timeout in ms
[bluetooth] elm327.services         01,02,03,06,07,  Allowed OBD2/UDS service IDs (hex, comma
                                    09,0A,19,1A,21,  separated), ``*`` = all. Other requests get the
                                    22,3E            negative response ``7F <sid> 11``.
=================================== ================ =================================================

The default service list only contains services that read data. It excludes for example
``04`` (clear DTCs), ``10`` (diagnostic session control), ``11`` (ECU reset), ``14`` (clear
diagnostic information), ``27`` (security access), ``2E`` (write data) and ``31`` (routine
control). Add a service only if you know what the app does with it.

The device name is read when Bluetooth starts. The other settings apply to the next
connection.


--------
Commands
--------

``bt elm327 status``
  Shows the Bluetooth state, the bonded devices, the pairing window, the client connection,
  the CAN bus and statistics.

``bt elm327 pair [<seconds>]``
  Opens the pairing window for new devices (default 120 seconds, 0 closes the window).

``bt status`` / ``bt clear [<mac>]``
  Show and remove bonded devices.


------------------------
ELM327 command support
------------------------

The emulation reports itself as ``ELM327 v1.5``. It supports CAN (ISO 15765-4) only.

**Requests:** hex requests with ISO-TP single and multi frame responses, 11 bit and 29 bit
CAN IDs. An odd last digit (expected number of responses) is ignored. An empty line repeats
the last command.

**Supported AT commands:** ``ATZ``, ``ATWS``, ``ATD``, ``ATI``, ``AT@1``, ``ATE0/1``, ``ATL0/1``,
``ATS0/1``, ``ATH0/1``, ``ATCAF0/1``, ``ATSP``, ``ATTP``, ``ATDP``, ``ATDPN``, ``ATSH`` (3, 6 or 8
digits), ``ATCP``, ``ATCRA``, ``ATAR``, ``ATST``, ``ATRV`` (from metric ``v.b.12v.voltage``),
``ATIGN``.

**Accepted without effect:** adaptive timing, flow control settings, CAN filters and masks,
memory, response and display settings, and similar commands (answer ``OK``). The OVMS poller
handles the ISO-TP flow control.

**Not supported:** monitoring (``ATMA``, ``ATMR``, ``ATMT``), buffer dump, STN extensions
(answer ``?``), non-CAN protocols.

**Request routing:**

- 11 bit: ``ATSH`` sets the request ID (default ``7DF``). The response ID is the ``ATCRA`` value,
  or the request ID + 8. A request to ``7DF`` is sent as a broadcast and accepts the responses
  from ``7E8`` … ``7EF``.
- 29 bit: ``ATSH`` with 6 digits uses the ``ATCP`` priority (default ``18``). Without ``ATCRA``,
  the response ID swaps the target and source bytes (``18DA10F1`` → ``18DAF110``). A functional
  request (``18DB33F1``) only receives the response of ``18DAF110``.
- In auto protocol mode, a 6 or 8 digit header selects the 29 bit variant of ``elm327.protocol``.

**Limitations:**

- With headers on (``ATH1``), a broadcast response is shown with the ID ``7E8``, also if
  another ECU sent it. Only the first response to a broadcast is returned.
- The request interrupts the poll list of the vehicle module. The poller continues the
  poll list after the request.
- A request must contain the complete PID for the service (for example 2 bytes for ``22``),
  otherwise the emulation answers ``?``.


----------------
Developer notes
----------------

The ELM327 command interpreter (``src/elm327_engine.*``) has no dependencies on the OVMS
framework. Its unit tests run on the build host::

  cd components/esp32bluetooth/tests
  make

For BLE tests without the target app, use a generic BLE app (for example nRF Connect):
subscribe to notifications on characteristic ``FFF1``, and write commands with a CR
(``0D``) at the end to characteristic ``FFF2``.
