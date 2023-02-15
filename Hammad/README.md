# PyGPSClient

[Current Status](#currentstatus) |
[Features](#features) |
[How to Use](#howtouse) |
[UBX Configuration](#ubxconfig) |
[NTRIP Client](#ntripconfig) |
[SPARTN Client](#spartnconfig) |
[Socket Server / NTRIP Caster](#socketserver) |
[GPX Track Viewer](#gpxviewer) |
[Installation](#installation) |
[Mapquest API Key](#mapquestapi) |
[User-defined Presets](#userdefined) |
[CLI Utilities](#cli) |
[Glossary of Terms](#glossary) |
[License](#license) |
[Author Information](#author)

PyGPSClient is a multi-platform graphical GNSS/GPS testing, diagnostic and UBX &copy; (u-blox &trade;) device configuration application written entirely in Python and tkinter.

![full app screenshot ubx](https://github.com/semuconsulting/PyGPSClient/blob/master/images/app.png?raw=true)

*Screenshot showing mixed-protocol stream from u-blox ZED-F9P receiver, using PyGPSClient's [NTRIP Client](#ntripconfig) to achieve >= 3cm accuracy*

The application can be installed using the standard `pip` Python package manager - see [installation instructions](#installation) below.

---
## <a name="currentstatus">Current Status</a>

![Status](https://img.shields.io/pypi/status/PyGPSClient)
![Release](https://img.shields.io/github/v/release/semuconsulting/PyGPSClient)
![Build](https://img.shields.io/github/actions/workflow/status/semuconsulting/PyGPSClient/main.yml?branch=master)
![Release Date](https://img.shields.io/github/release-date/semuconsulting/PyGPSClient)
![Last Commit](https://img.shields.io/github/last-commit/semuconsulting/PyGPSClient)
![Contributors](https://img.shields.io/github/contributors/semuconsulting/PyGPSClient.svg)
![Open Issues](https://img.shields.io/github/issues-raw/semuconsulting/PyGPSClient)

Sphinx API Documentation in HTML format is available at [https://www.semuconsulting.com/pygpsclient](https://www.semuconsulting.com/pygpsclient).

Contributions welcome - please refer to [CONTRIBUTING.MD](https://github.com/semuconsulting/PyGPSClient/blob/master/CONTRIBUTING.md).

[Bug reports](https://github.com/semuconsulting/PyGPSClient/blob/master/.github/ISSUE_TEMPLATE/bug_report.md) and [Feature requests](https://github.com/semuconsulting/PyGPSClient/blob/master/.github/ISSUE_TEMPLATE/feature_request.md) - please use the templates provided.

This is an independent project and we have no affiliation whatsoever with u-blox.

---
## <a name="features">Features</a>

1. Runs on any platform which supports a Python 3 interpreter (>=3.7) and tkinter (>=8.6) GUI framework, including Windows, MacOS, Linux and Raspberry Pi OS.
1. Supports NMEA, UBX, RTCM3, NTRIP and SPARTN protocols.
1. Capable of reading from a variety of GNSS data streams: Serial (USB / UART), Socket (TCP / UDP) and binary datalog file.
1. Configurable GUI with selectable and resizeable widgets.
1. Expandable banner widget showing key navigation information.
1. Serial console widget showing data stream in either parsed, binary or hexadecimal format.
1. User-configurable color tagging of selected console entries.
1. Skyview widget showing current satellite visibility and position (elevation / azimuth). Satellite icon borders are colour-coded to distinguish between different GNSS constellations.
1. Graphview widget showing current satellite reception (signal-to-noise ratio).
1. Mapview widget with location marker, showing either a static Mercator world map, or an optional dynamic web-based map downloaded via a MapQuest API (*requires an Internet connection and free 
[MapQuest API Key](https://developer.mapquest.com/user/login/sign-up)*).
1. Spectrum widget showing spectrum analysis chart from MON-SPAN message (*UBX MON-SPAN messages must be enabled or polled*).
1. Data logging in parsed, binary, hexadecimal string and tabulated hexadecimal formats (NB. only binary datalogs can be re-read by PyGPSClient's parser).
1. Track recording in GPX format.
1. [UBX Configuration Dialog](#ubxconfig), with the ability to send a variety of UBX CFG configuration commands to u-blox GNSS devices. This includes the facility to add **user-defined commands or command sequences** - see instructions under [user-defined presets](#userdefined) below. While not intended to be a direct replacement, the application supports much of the UBX configuration functionality in u-blox's Windows-only [u-center &copy;](https://www.u-blox.com/en/product/u-center) tool.
1. [NTRIP Client](#ntripconfig) facility with the ability to connect to a specified NTRIP caster, parse the incoming RTCM3 data and feed this data to a compatible GNSS receiver (*requires an Internet connection and access to a suitable NTRIP caster*).
1. [SPARTN Client](#spartnconfig) facility with the ability to configure an IP or L-Band SPARTN Correction source and SPARTN-compatible GNSS receiver (e.g. ZED-F9P) and pass the incoming correction data to the GNSS receiver (*requires an Internet connection and access to a SPARTN location service*).
1. [Socket Server / NTRIP Caster](#socketserver) facility with two modes of operation: (a) open, unauthenticated Socket Server or (b) NTRIP Caster.
1. [GPX Track Viewer](#gpxviewer) utility with elevation and speed profiles and track metadata (*requires an Internet connection and free 
[MapQuest API Key](https://developer.mapquest.com/user/login/sign-up)*).

---
## <a name="howtouse">How to Use</a>

* To connect to a listed serial device, select the device from the listbox, set the appropriate serial connection parameters and click 
![connect icon](/pygpsclient/resources/usbport-1-24.png). The application will endeavour to pre-select a recognised GNSS/GPS device but this is platform and device dependent. Press the ![refresh](/pygpsclient/resources/iconmonstr-refresh-6-16.png) button to refresh the list of connected devices at any point. `Rate bps` (baud rate) is typically the only setting that might need adjusting, but tweaking the `timeout` setting may improve performance on certain platforms. The `Msg Mode` parameter defaults to `GET` i.e., periodic or poll response messages *from* a receiver. If you wish to parse streams of command or poll messages being sent *to* a receiver, set the `Msg Mode` to `SET` or `POLL`.
* A default user-defined serial port can also be passed via the environment variable `PYGPSCLIENT_USERPORT` or as a command line argument `--userport /dev/tty12345`.
* To connect to a TCP or UDP socket, enter the server URL and port, select the protocol (defaults to TCP) and click 
![connect socket icon](/pygpsclient/resources/ethernet-1-24.png).
* To stream from a previously-saved <a name="filestream">binary datalog file</a>, click 
![connect-file icon](/pygpsclient/resources/binary-1-24.png) and select the file type (`*.log, *.ubx, *.*`) and path. PyGPSClient datalog files will be named e.g. `pygpsdata-20220427114802.log`, but any binary dump of an GNSS receiver output is acceptable, including `*.ubx` files produced by u-center.
* To disconnect from the data stream, click
![disconnect icon](/pygpsclient/resources/iconmonstr-media-control-50-24.png).
* To display the UBX Configuration Dialog (*only functional when connected to a UBX GNSS device via serial port*), click
![gear icon](/pygpsclient/resources/iconmonstr-gear-2-24.png), or go to Menu..Options.
* To display the NTRIP Client Configuration Dialog (*requires Internet connection*), click
![gear icon](/pygpsclient/resources/iconmonstr-antenna-6-24.png), or go to Menu..Options..NTRIP Configuration Dialog.
* To display the SPARTN Client Configuration Dialog (*may require Internet connection*), go to Menu..Options..SPARTN Configuration Dialog.
* To expand or collapse the banner or serial port configuration widgets, click the ![expand icon](/pygpsclient/resources/iconmonstr-arrow-80-16.png)/![expand icon](/pygpsclient/resources/iconmonstr-triangle-1-16.png) buttons.
* To show or hide the various widgets, go to Menu..View and click on the relevant hide/show option.
* Protocols Shown - Select which protocols to display; NMEA, UBX and/or RTCM3 (NB: this only changes the displayed protocols - to change the actual protocols output by the receiver, use the CFG-PRT command).
* Console Display - Select from parsed, binary or hexadecimal formats.
* Tags - Turn console color tagging on or off. User-configurable color tags are loaded from a file "colortags" (lower case, no extension) in the user's home directory - see example provided. NB: color tagging does impose a small performance overhead - turning it off will improve console response times at very high transaction rates.
* Degrees Format and Units - Change the displayed degree and unit formats.
* Zoom - Change the web map scale (any change will take effect at the next map refresh, indicated by a small timer icon at the top left of the panel).
* Show Legend - Turn the graph legend on or off.
* Show Unused Satellites - Include or exclude satellites that are not used in the navigation solution (e.g. because their signal level is too low) from the graph and sky view panels.
* DataLogging - Turn Data logging in the selected format on or off. You will be prompted to select the directory into which timestamped log files are saved (NB. only binary datalogs can be re-read by PyGPSClient's parser).
* GPX Track - Turn track recording (in GPX format) on or off. You will be prompted to select the directory into which timestamped track files are saved.
* GPX Track Viewer - Display the GPX Track Viewer dialog via Menu..Options..GPX Track Viewer. Click the load button to load a GPX file and display the map and profile. Click the redraw button to redraw the map.
* Socket / NTRIP Server - Turn Socket / NTRIP Server on or off.
* UBX Configuration - Opens UBX configuration panel (only functional with u-blox devices).
* NTRIP Client Configuration - Opens NTRIP Client configuration panel.

---
### <a name="ubxconfig">UBX Configuration Facilities</a>

![ubxconfig widget screenshot](https://github.com/semuconsulting/PyGPSClient/blob/master/images/ubxconfig_widget.png?raw=true)

**Pre-Requisites:**

- u-blox GNSS receiver e.g. u-blox NEO-M8S, ZED-F9P, connected to the workstation via USB or UART port.

**Instructions:**

The UBX Configuration Dialog currently supports the following UBX configuration panels:
1. Version panel shows current device hardware/firmware versions (*via MON-VER and MON-HW polls*).
1. CFG Configuration Load/Save/Record facility. This allows users to record ![record icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-record-24.png?raw=true) a sequence of UBX CFG configuration commands, and to save ![save icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-save-14-24.png?raw=true) this recording to a file (as binary CFG-* messages). Saved files can be reloaded ![load icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-folder-18-24.png?raw=true) and the configuration commands replayed ![play icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-arrow-12-24.png?raw=true). This provides a means to easily reproduce a given sequence of configuration commands, or copy a saved configuration between compatible devices. Files saved using the [ubxsave](#ubxsave) CLI utility can also be reloaded and replayed. **Tip:** The contents of a binary config file can be reviewed using PyGPSClient's [file streaming facility](#filestream), *BUT* remember to set the `Msg Mode` in the Settings panel to `SET` rather than the default `GET` ![msgmode capture](https://github.com/semuconsulting/PyGPSClient/blob/master/images/msgmode.png?raw=true).
1. Protocol Configuration panel (CFG-PRT) sets baud rate and inbound/outbound protocols across all available ports.
1. Solution Rate panel (CFG-RATE) sets navigation solution interval in ms (e.g. 1000 = 1/second) and measurement ratio (ratio between the number of measurements and the number of navigation solutions, e.g. 5 = five measurements per navigation solution).
1. For each of the panels above, clicking anywhere in the panel background will refresh the displayed information with the current configuration.
1. Message Rate panel (CFG-MSG) sets message rates per port for UBX and NMEA messages. Message rate is relative to navigation solution frequency e.g. a message rate of '4' means 'every 4th navigation solution'.
1. Other configuration panel (CFG-*) providing structured updates for a range of legacy CFG- configuration commands for pre-Generation 9+ devices. Note: 'X' (byte) type attributes can be entered as integers or hexadecimal strings e.g. 522125312 or 0x1f1f0000.
1. Configuration Interface widget (CFG-VALSET, CFG-VALDEL and CFG-VALGET) queries and sets configuration for [Generation 9+ devices](https://github.com/semuconsulting/pyubx2#configinterface) e.g. NEO-M9, ZED-F9P, etc.
1. Preset Commands widget supports a variety of preset and user-defined commands - see [user defined presets](#userdefined)

An icon to the right of each 'SEND' 
![send icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-arrow-12-24.png?raw=true) button indicates the confirmation status of the configuration command; 
(pending i.e. awaiting confirmation ![pending icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-time-6-24.png?raw=true), 
confirmed ![confirmed icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-check-mark-8-24.png?raw=true) or 
warning ![warning icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-warning-1-24.png?raw=true)). 

**Note:**
* Poll and Acknowledgement responses can take several seconds at high message transmission rates, or be discarded altogether if the device's transmit buffer is full (*txbuff-alloc error*). To ensure timely responses, try increasing the baud rate and/or temporarily reducing transmitted message rates using the configuration commands provided.
* A warning icon (typically accompanied by an ACK-NAK response) is usually an indication that one or more of the commands sent is not supported by your receiver.

---
### <a name="ntripconfig">NTRIP Client Facilities</a>

![ntrip config widget screenshot](https://github.com/semuconsulting/PyGPSClient/blob/master/images/ntripconfig_widget.png?raw=true)

The NTRIP Configuration utility allows users to receive and process NTRIP RTK Correction data from an NTRIP caster to achieve cm level location accuracy. The facility can be accessed by clicking the ![NTRIP Client button](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-antenna-4-24.png?raw=true) or selecting Menu..Options..NTRIP Configuration Dialog. 

**Pre-Requisites:**

- NTRIP-compatible GNSS receiver e.g. u-blox ZED-F9P
- Internet access
- URL of NTRIP caster
- Login credentials for the NTRIP caster (where required)
- Name of local MOUNTPOINT (if not using PyGPSClient's automatic mountpoint locator)

**Instructions:**

1. Enter the required NTRIP server URL (or IP address) and port (defaults to 2101). For services which require authorisation, enter your login username and password.
1. To retrieve the sourcetable, leave the mountpoint field blank and click connect (*response may take a few seconds*). The required mountpoint may then be selected from the list, or entered manually. Where possible, `PyGPSClient` will automatically identify the closest mountpoint to the current location.
1. For NTRIP services which require client position data via NMEA GGA sentences, select the appropriate sentence transmission interval in seconds. The default is 'None' (no GGA sentences sent). A value of 10 or 60 seconds is typical.
1. If GGA sentence transmission is enabled, GGA sentences can either be populated from live navigation data (*assuming a receiver is connected and outputting valid position data*) or from fixed reference settings entered in the NTRIP configuration panel (latitude, longitude, elevation and geoid separation - all four reference settings must be provided).
1. To connect to the NTRIP server, click ![connect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-media-control-48-24.png?raw=true). To disconnect, click ![disconnect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-media-control-50-24.png?raw=true).
1. If NTRIP data is being successfully received, the banner '**dgps:**' status indicator should change to 'YES' and indicate the age and reference station of the correction data (where available) ![dgps status](https://github.com/semuconsulting/PyGPSClient/blob/master/images/dgps_status.png?raw=true). Note that DGPS status is typically maintained for up to 60 seconds after loss of correction signal.
1. Some NTRIP services may output RTCM3 correction messages at a high rate, flooding the GUI console display. To suppress these messages in the console, de-select the 'RTCM' option in 'Protocols Shown' - the RTCM3 messages will continue to be processed in the background.

Below is a illustrative NTRIP DGPS data log, showing:
* Outgoing NMEA GPGGA (client position) sentence.
* Incoming RTCM3 correction messages; in this case - 1006 (Ref station ARP (*DF003=2690*) with antenna height), 1008 (Antenna descriptor), 1033 (Receiver descriptor), 1075 (GPS MSM5), 1085 (GLONASS MSM5), 1095 (Galileo MSM5), 1125 (BeiDou MSM5) and 1230 (GLONASS Code-Phase Biases)
* Corresponding UBX RXM-RTCM acknowledgements generated by the u-blox ZED-F9P receiver, confirming message type, valid checksum (*crcFailed=0*), successful use (*msgUsed=2*) and reference station ARP (*refStation=2690*). 

![ntrip console screenshot](https://github.com/semuconsulting/PyGPSClient/blob/master/images/ntrip_consolelog.png?raw=true)

**NB:** Please respect the terms and conditions of any remote NTRIP service used with this facility. For testing or evaluation purposes, consider deploying a local [SNIP LITE](https://www.use-snip.com/download/) server. *Inappropriate use of an NTRIP service may result in your account being blocked*.

---
### <a name="spartnconfig">SPARTN Client Facilities</a>

**BETA FEATURE**

![spartn config widget screenshot](https://github.com/semuconsulting/PyGPSClient/blob/master/images/spartnconfig_widget.png?raw=true)

The SPARTN Configuration utility allows users to receive and process SPARTN RTK Correction data from an IP or L-Band source to achieve cm level location accuracy. It provides three independent configuration sections, one for IP Correction (MQTT), one for L-Band Correction (e.g. D9S) and a third for the GNSS receiver (e.g. F9P). 

**NB:** the SPARTN client implements skeleton `SPARTNReader` and `SPARTNMessage` classes. The `SPARTNReader` class will parse individual SPARTN messages from any binary stream containing
*solely* SPARTN data e.g. an MQTT `/pp/ip` topic. The `SPARTNMessage` class does not currently perform a full decode of SPARTN protocol messages; it basically decodes just enough information to identify message type/subtype, payload length and other key metadata.

The facility can be accessed by clicking the ![SPARTN Client button](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-antenna-3-24.png?raw=true) or selecting Menu..Options..NTRIP Configuration Dialog. . The dialog must remain open while the facility is in use.

**Pre-Requisites:**

1. IP Correction (MQTT Client):

    - Internet access
    - Subscription to relevant MQTT SPARTN location service e.g. u-blox / Thingstream PointPerfect, which should provide the following details:
    - Server e.g. `pp.services.u-blox.com`
    - Client ID (which can be stored as environment variable `MQTTCLIENTID`)
    - Encryption certificate (`*.crt`) and key (`*.pem`). If these are placed in the user's HOME directory, PyGPSClient will find them automatically.
    - Region code e.g. `us`, `eu`
    - A list of published topics. These typically include `/pp/ip/region` (binary SPARTN correction data for the selected region), `/pp/ubx/mga` (UBX MGA AssistNow ephemera data for each constellation) and `/pp/ubx/0236/ip` (UBX RXM-SPARTNKEY messages containing the decryption keys to be uploaded to the GNSS receiver).

2. L-BAND Correction (D9* Receiver):

    - SPARTN L-Band correction receiver e.g. u-blox NEO-D9S
    - Suitable Immarsat L-band antenna (NB: standard GNSS antenna may not be suitable)
    - Subscription to L-Band location service e.g. u-blox / Thingstream PointPerfect, which should provide the following details:
    - L-Band frequency; search window; use_serviceid?; use_descrambling?; use_prescrambling?; Service ID; Data Rate; Descrambler Init; Unique Word

3. GNSS Receiver:

    - SPARTN-compatible GNSS receiver e.g. u-blox ZED-F9P
    - Subscription to either IP and/or L-Band location service(s) e.g. e.g. u-blox / Thingstream PointPerfect, which should provide the following details:
    - Current and Next Encryption Keys in hexadecimal format; Valid From dates in YYYYMMDD format (keys normally valid for 4 week period).

**Instructions:**

#### IP Correction Configuration (MQTT)

1. Enter your MQTT Client ID (or set it up beforehand as environment variable `MQTTCLIENTID`).
1. Select the path to your MQTT `*.crt` and `*.pem` files (PyGPSClient will use the user's HOME directory by default).
1. Select the required topics:
    - IP (required) - this is the raw SPARTN correction data.
    - MGA (optional) - this is Assist Now data as a UBX MGA message.
    - SPARTNKEY (optional, recommended) - this is the SPARTN encryption keys as a UBX RXM-SPARTNKEY message.
1. To connect to the MQTT server, select the Server URL, Client ID, Region and Topics and click ![connect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/ethernet-1-24.png?raw=true). To disconnect, click ![disconnect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-media-control-50-24.png?raw=true).

#### L-Band Correction Configuration (D9*)

1. To connect to the Correction receiver, select the receivers's port from the SPARTN dialog's Serial Port listbox and click ![connect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/usbport-1-24.png?raw=true). To disconnect, click ![disconnect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-media-control-50-24.png?raw=true).
1. Select the required Output Port - this is the port used to connect the Correction receiver to the GNSS receiver e.g. UART2 or I2C.
1. If both Correction and GNSS receivers are connected to the same PyGPSClient workstation (e.g. via separate USB ports), it is possible to run the utility in Output Port = 'Passthough' mode, whereby the output data from the Correction receiver (UBX `RXM-PMP` messages) will be automatically passed through to the GNSS receiver by PyGPSClient, without the need to connect the two externally.
1. To enable INF-DEBUG messages, which give diagnostic information about current L-Band reception, click 'Enable Debug?'.
1. Once connected, click ![send button](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-arrow-12-24.png?raw=true) to upload the configuration. The utility will send the relevant configuration command(s) to the Correction receiver and poll for an acknowledgement.

#### GNSS Receiver Configuration (F9*)

1. Connect to the GNSS receiver first by clicking ![connect icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/usbport-1-24.png?raw=true) on the main PyGPSClient settings panel.
1. Enter the current and next keys in hexadecimal format e.g. `0102030405060708090a0b0c0d0e0f10`. The keys are normally 16 bytes long, or 32 hexadecimal characters.
1. Enter the supplied Valid From dates in `YYYYMMDD` format. **NB:** These are *Valid From* dates rather than *Expiry* dates. If the location service provides Expiry dates, subtract 4 weeks from these to get the Valid From dates.
1. NB: if you are subscribing to the MQTT SPARTNKEY topic, the key and date details will be entered automatically on reeipt of an MQTT SPARTNKEY message.
1. Select 'Upload keys', 'Configure receiver' and 'Disable NMEA' options as required.
1. Click ![send button](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-arrow-12-24.png?raw=true) to upload the configuration. The utility will send the relevant configuration command(s) to the receiver and poll for an acknowledgement.

---
### <a name="socketserver">Socket Server / NTRIP Caster Facilities</a>

The Socket Server / NTRIP Caster facility is capable of operating in either of two modes;
1. SOCKET SERVER - an open, unauthenticated TCP socket server available to any socket client including, for example, another instance of PyGPSClient or `gnssdump` (CLI utility installed with `pygnssutils`) running on another machine. In this mode it will broadcast the host's currently connected GNSS data stream (NMEA, UBX, RTCM3). The default port is 50010.
2. NTRIP CASTER - a simple implementation of an authenticated NTRIP caster available to any NTRIP client including, for example, the PyGPSClient NTRIP Client facility or `gnssntripclient` (CLI utility installed with `pygnssutils`). Login credentials for the NTRIP caster are set via host environment variables `PYGPSCLIENT_USER` and `PYGPSCLIENT_PASSWORD`. The default port is 2101.

**Pre-Requisites:**

Running in NTRIP caster mode is predicated on the host being connected to an RTK-compatible GNSS receiver **operating in Base Station mode** (either `SURVEY_IN` or `FIXED`) and outputting the requisite RTCM3 message types (1005, 1077, 1087, 1097, 1127 and 1230). In the case of the u-blox ZED-F9P receiver, for example, this is set using the `CFG-TMODE*` and `CFG-MSGOUT-RTCM*` configuration interface parameters available via the [UBX Configuration panel](#ubxconfig) - refer to the Integration Manual and Interface Specification for further details. PyGPSClient does *not* configure the receiver automatically, though an example configuration script for the u-blox ZED-F9P receiver may be found at [/examples/f9p_basestation.py](https://github.com/semuconsulting/PyGPSClient/blob/master/examples/f9p_basestation.py). This script can also be used to generate user-defined preset command strings suitable for copying and pasting into a `ubxpresets` file (see [User Defined Presets](#userdefined) below).

**Instructions:**

The server/caster binds to the host address '0.0.0.0' i.e. all available IP addresses on the host machine. It may be necessary to add a firewall rule on the host and/or client machine to allow remote traffic on the specified port.

A label on the settings panel indicates the number of connected clients, and the server/caster status is indicated in the topmost banner: open with no clients: ![transmit icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-noclient-10-24.png?raw=true), open with clients: ![transmit icon](https://github.com/semuconsulting/PyGPSClient/blob/master/pygpsclient/resources/iconmonstr-transmit-10-24.png?raw=true).

---
### <a name="gpxviewer">GPX Track Viewer</a>

![gpxviewer screenshot](https://github.com/semuconsulting/PyGPSClient/blob/master/images/gpxviewer.png?raw=true)

*GPX Track Viewer screenshot*

---
## <a name="installation">Installation</a>

In the following, `python3` & `pip` refer to the Python 3 executables. You may need to type 
`python` or `pip3`, depending on your particular environment. It is also recommended that 
the Python 3 scripts (bin) and site_packages directories are included in your PATH 
(*many standard Python 3 installation packages will do this automatically*).

### Platform Dependencies

On Windows and MacOS, pip, tkinter and the necessary imaging libraries are generally packaged with Python 3.  On some Linux distributions like Ubuntu 18+ and Raspberry Pi OS, they may need to be installed separately, e.g.:

```shell
sudo apt install python3-pip python3-tk python3-pil python3-pil.imagetk
```

*NB:* If you're compiling the latest version of Python 3 from source, you may also need to install tk-dev (or a similarly named package e.g. tk-devel) first. Refer to http://wiki.python.org/moin/TkInter for further details.

```shell
sudo apt install tk-dev
```

Ideally the platform screen resolution will be at least 1400 x 900, though the main application window is resizeable.

### User Privileges

To access the serial port on most Linux platforms, you will need to be a member of the 
`tty` and/or `dialout` groups. Other than this, no special privileges are required.

```shell
usermod -a -G tty myuser
```

### 1. Install using pip

![Python version](https://img.shields.io/pypi/pyversions/PyGPSClient.svg?style=flat)
[![PyPI version](https://img.shields.io/pypi/v/PyGPSClient.svg?style=flat)](https://pypi.org/project/PyGPSClient/)
![PyPI downloads](https://img.shields.io/pypi/dm/PyGPSClient.svg?style=flat)

The easiest way to install the latest version of `PyGPSClient` is with
[pip](http://pypi.python.org/pypi/pip/):

```shell
python3 -m pip install --upgrade PyGPSClient
```

If required, `PyGPSClient` can also be installed into a virtual environment, e.g.:

```shell
python3 -m pip install --user --upgrade virtualenv
python3 -m virtualenv env
source env/bin/activate (or env\Scripts\activate on Windows)
(env) python3 -m pip install --upgrade PyGPSClient
...
deactivate
```

To run the application, if the Python 3 scripts (bin) directory is in your PATH, simply type (all lowercase): 
```shell
pygpsclient
```

Optionally, a user-defined serial port can be passed as a keyword argument, e.g.
```shell
pygpsclient --userport /dev/tty12345
```

If desired, you can add a shortcut to this command to your desktop or favourites menu.

Alternatively, if the Python 3 site_packages are in your PATH, you can type (all lowercase): 
```shell
python3 -m pygpsclient
```

**NB:** If the Python 3 scripts (bin) or site_packages directories are *not* in your PATH, you will need to add the fully-qualified path to `pygpsclient` in the commands above.

**Tip:** to find the site_packages location, type:
```shell
python3 -m pip show PyGPSClient
``` 
and look for the `Location:` entry in the response, e.g.

- Linux: `Location: /home/username/.local/lib/python3.11/site-packages`
- MacOS: `Location: /Library/Frameworks/Python.framework/Versions/3.11/lib/python3.11/site-packages`
- Windows: `Location: c:\users\username\appdata\roaming\python\python311\lib\site-packages`

**Tip:** To create an application launcher for linux distributions like Ubuntu, create a text file named `pygpsclient.desktop` with the following content (*edited for your particular environment*) and copy this to the `/home/user/.local/share/applications` folder, e.g.

```
[Desktop Entry]
Type=Application
Terminal=false
Name=PyGPSClient
Icon=/home/user/.local/lib/python3.11/site-packages/pygpsclient/resources/pygpsclient.ico
Exec=/home/user/.local/bin/pygpsclient
```

### 2. Manual installation

See [requirements.txt](requirements.txt).

The following Python libraries are required (these will be installed automatically if using pip to install PyGPSClient):

```shell
python3 -m pip install --upgrade pygnssutils pyserial Pillow requests paho-mqtt
```

To install PyGPSClient manually, download and unzip this repository and run:

```shell
python3 -m /path_to_folder/foldername/pygpsclient
```

e.g. if you downloaded and unzipped to a folder named `PyGPSClient-1.3.16`, run: 

```shell
python3 -m /path_to_folder/PyGPSClient-1.3.16/pygpsclient
```

---
## <a name="mapquestapi">MapQuest API Key</a>

To use the optional dynamic web-based mapview or GPX Track Viewer facilities, you need to request and install a 
free [MapQuest API key](https://developer.mapquest.com/user/login/sign-up) (*no payment details required*).
The free edition of this API allows for up to 15,000 transactions/month (roughly 500/day) on a non-commercial basis.
For this reason, the map refresh rate is intentionally limited to 1/minute* to avoid exceeding the free transaction
limit under normal use. **NB:** this facility is *not* intended to be used for real time navigational purposes.

Once you have received the API key (a 32-character alphanumeric string), you can either:

1. create an environment variable named `MQAPIKEY` (all upper case) and set this to the API key value. It is recommended 
that this is a User variable rather than a System/Global variable.
2. copy it to a file named `mqapikey` (lower case, no extension) and place this file in the user's home directory.

*The web map refresh rate can be amended if required by changing the MAP_UPDATE_INTERVAL constant in `globals.py`.

---
## <a name="userdefined">User Defined Presets</a>

The UBX Configuration Dialog includes the facility to send user-defined UBX configuration messages or message sequences to the receiver. These can be set up by adding
appropriate comma-delimited message descriptions and payload definitions to a file named `ubxpresets` (lower case, no extension), and then placing this file in the user's home directory. The message definition comprises a free-format text description (*avoid embedded commas*) 
followed by one or more [pyubx2 UBXMessage constructors](https://pypi.org/project/pyubx2/), i.e. 
1. message class as a string e.g. `CFG` (must be a valid class from pyubx2.UBX_CLASSES)
2. message id as a string e.g. `CFG-MSG` (must be a valid id from pyubx2.UBX_MSGIDS)
3. payload as a hexadecimal string e.g. `f004010100010100` (leave blank for null payloads e.g. most POLL messages)
4. mode as an integer (`1` = SET, `2` = POLL)

(payload as hex string can be obtained from a `UBXMessage` created using the [pyubx2 library](https://pypi.org/project/pyubx2/) thus: ```msg.payload.hex()```)

Multiple commands can be concatenated on a single line. Illustrative examples are shown below:

```
Stop GNSS, CFG, CFG-RST, 00000800, 1
Start GNSS, CFG, CFG-RST, 00000900, 1
Enable NMEA UBX00 & UBX03 sentences, CFG, CFG-MSG, f100010100010100, 1, CFG, CFG-MSG, f103010100010100, 1
Poll NEO-9 UART1/2 baud rates, CFG, CFG-VALGET, 000000000100524001005340, 2
Poll NEO-9 Message Rates, CFG, CFG-VALGET, 00000000ffff9120, 2, CFG, CFG-VALGET, 00004000ffff9120, 2, CFG, CFG-VALGET, 00008000ffff9120, 2
Set ZED-F9P to Base Station Survey-In Mode (1m accuracy), CFG, CFG-VALSET, 000100008b00912001c002912001cf02912001d4029120011b03912001d902912001060391200111000340e8030000100003405a0000000100032001, 1
Set ZED-F9P to Base Station Survey-In Mode (1cm accuracy), CFG, CFG-VALSET, 000100008b00912001c002912001cf02912001d4029120011b03912001d9029120010603912001110003400a000000100003405a0000000100032001, 1
Poll Receiver Software Version, MON, MON-VER, , 2
Poll Datum, CFG, CFG-DAT, , 2
Poll GNSS config, CFG, CFG-GNSS, , 2
Poll NMEA config, CFG, CFG-NMEA, , 2
Poll Satellite-based Augmentation, CFG, CFG-SBAS, , 2
Poll Receiver Management, CFG, CFG-RXM, , 2
Poll Navigation Mode, CFG, CFG-NAV5, , 2
Poll Expert Navigation mode, CFG, CFG-NAVX5, , 2
Poll Geofencing, CFG, CFG-GEOFENCE, , 2
Poll Timepulse, CFG, CFG-TP5, , 2
Set NEO-M8T Timepulse to 1 Hz, CFG, CFG-TP5, 000100003200000001000000010000003200000032000000000000006f000000, 1
Set NEO-M8T Timepulse to 8 MHz, CFG, CFG-TP5, 000100003200000000127a0000127a003200000032000000000000006f000000, 1
Limit NMEA GNSS to GPS only, CFG, CFG-NMEA, 0040000276000000000000010000000000000000, 1
Limit NMEA GNSS to GLONASS only, CFG, CFG-NMEA, 0040000257000000000000010000000000000000, 1
Set NMEA GNSS to ALL, CFG, CFG-NMEA, 0040000200000000000000010000000000000000, 1
Limit UBX GNSS to GPS only, CFG, CFG-GNSS, 0020200700081000010001010101030000000101020408000000010103081000000001010400080000000103050003000000010506080E0000000101, 1
Limit UBX GNSS to GLONASS only, CFG, CFG-GNSS, 0020200700081000000001010101030000000101020408000000010103081000000001010400080000000103050003000000010506080E0001000101, 1
Set UBX GNSS to ALL, CFG, CFG-GNSS, 0020200700081000010001010101030001000101020408000000010103081000000001010400080000000103050003000100010506080E0001000101, 1
FORCE COLD RESTART !*** Expect ClearCommError ***!, CFG, CFG-RST, ffff0100, 1
```

---
## <a name="cli">Command Line Utilities</a>

The `pygnssutils` library which underpins many of the functions in `PyGPSClient` also incorporates command line versions of these functions:

1. `gnssdump` CLI utility. This is essentially a configurable input/output wrapper around the [`pyubx2.UBXReader`](https://github.com/semuconsulting/pyubx2#reading) class with flexible message formatting and filtering options for NMEA, UBX and RTCM3 protocols.
1. `gnssserver` CLI utility. This implements a TCP Socket Server for GNSS data streams which is also capable of being run as a simple NTRIP Server.
1. `gnssntripclient` CLI utility. This implements
a simple NTRIP Client which receives RTCM3 correction data from an NTRIP Server and (optionally) sends this to a
designated output stream.
1. <a name="ubxsave">`ubxsave` CLI utility</a>. This saves a complete set of configuration data from any Generation 9+ u-blox device (e.g. NEO-M9N or ZED-F9P) to a file. The file can then be reloaded to any compatible device using the `ubxload` utility.
1. `ubxload` CLI utility. This reads a file containing binary configuration data and loads it into any compatible Generation 9+ u-blox device (e.g. NEO-M9N or ZED-F9P).
1. `ubxsetrate` CLI utility. A simple utility which sets NMEA or UBX message rates on u-blox GNSS receivers.

For further details, refer to the `pygnssutils` homepage at [https://github.com/semuconsulting/pygnssutils](https://github.com/semuconsulting/pygnssutils).

---
## <a name="glossary">Glossary of Terms</a>

For a general overview of GNSS, DGPS, RTK, NTRIP and SPARTN technologies and terminology, refer to [GNSS Positioning - A Reviser](https://www.semuconsulting.com/gnsswiki/).

* ACC - accuracy of location in real units (hacc - horizontal, vacc - vertical). Note that location accuracy is not directly provided via the standard NMEA message set, but is available from UBX messages e.g. NAV-POSLLH, NAV-PVT and some proprietary NMEA messages e.g. UBX00.
* BEI - [BeiDou Navigation Satellite System](https://en.wikipedia.org/wiki/BeiDou).
* DGPS - [Differential GPS](https://en.wikipedia.org/wiki/Differential_GPS).
* DOP - [dilution of precision](https://gisgeography.com/gps-accuracy-hdop-pdop-gdop-multipath/) (pdop - overall position, hdop - horizontal, vdop - vertical).
* GAL - [Galileo Satellite Navigation](https://en.wikipedia.org/wiki/Galileo_(satellite_navigation)).
* GLO - [GLONASS, Global Navigation Satellite System](https://en.wikipedia.org/wiki/GLONASS).
* GNSS - [Global Navigation Satellite System](https://en.wikipedia.org/wiki/Satellite_navigation).
* GPS - [Global Positioning System](https://en.wikipedia.org/wiki/Global_Positioning_System).
* IME - [IMES, Indoor MEssaging System](https://www.gpsworld.com/wirelessindoor-positioningopening-up-indoors-11603/).
* NTRIP - [Networked Transport of RTCM via Internet Protocol](https://en.wikipedia.org/wiki/Networked_Transport_of_RTCM_via_Internet_Protocol).
* PRN - [pseudo-random noise number](https://www.gps.gov/technical/prn-codes/) (code that each satellite transmits to differentiate itself from other satellites in the active constellation).
* QZSS - [Quasi-Zenith Satellite System](https://en.wikipedia.org/wiki/Quasi-Zenith_Satellite_System).
* RTCM(3) - [RTCM 10403.n Differential GNSS Services standard](https://rtcm.myshopify.com/collections/differential-global-navigation-satellite-dgnss-standards/products/rtcm-10403-2-differential-gnss-global-navigation-satellite-systems-services-version-3-february-1-2013).
* RTK - [Real-time Kinematics](https://en.wikipedia.org/wiki/Real-time_kinematic_positioning).
* SBA - [SBAS, Satellite-based Augmentation System](https://en.wikipedia.org/wiki/GNSS_augmentation#Satellite-based_augmentation_system).
* SIP - satellites used in position solution.
* SIV - satellites in view.
* SPARTN - [Secure Position Augmentation for Real Time Navigation](https://www.spartnformat.org/).
* SV(N) - [space vehicle number](https://en.wikipedia.org/wiki/List_of_GPS_satellites) (serial number assigned to each satellite).
* UTC - coordinated universal time.

---
## <a name="license">License</a>

![License](https://img.shields.io/github/license/semuconsulting/PyGPSClient.svg)

BSD 3-Clause License

Copyright &copy; 2020, SEMU Consulting
All rights reserved.

Application icons from [iconmonstr](https://iconmonstr.com/license/) &copy;.

---
## <a name="author">Author Information</a>

semuadmin@semuconsulting.com

`PyGPSClient` is maintained entirely by unpaid volunteers. It receives no funding from advertising or corporate sponsorship. If you find the utility useful, a small donation would be greatly appreciated!

[![Donations](https://www.paypalobjects.com/en_GB/i/btn/btn_donate_LG.gif)](https://www.paypal.com/donate/?business=UL24WUA4XHNRY&no_recurring=0&item_name=The+SEMU+GNSS+Python+libraries+are+maintained+entirely+by+unpaid+volunteers.+All+donations+are+greatly+appreciated.&currency_code=GBP)
