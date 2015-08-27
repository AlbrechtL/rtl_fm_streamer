RTL SDR FM Streamer
===================
Turns your Realtek RTL2832 based DVB dongle into a FM radio receiver.

Description
-----------
RTL SDR FM Streamer is a small tool to stream FM radio by using a DVB-T dongle to a client e.g Kodi, VLC or mplayer.

The DVB-T dongle has to be based on the Realtek RTL2832U.
See [http://sdr.osmocom.org/trac/wiki/rtl-sdr](http://sdr.osmocom.org/trac/wiki/rtl-sdr) for more RTL SDR details.

Usage
-----
Default port: 2346

    $ ./rtl_fm_streamer

Options
-------
The options "-P" defines the port where the HTTP server is listen on.

e.g. port 12345

    $ ./rtl_fm_streamer -P 12345

Streaming
---------
To connect to the server you can use KODI, VLC or mplayer. Just connect to the URL
mono: "http://IP:port/FrequencyInHerz"
mono: "http://IP:port/FrequencyInHerz/0"
stereo: "http://IP:port/FrequencyInHerz/1"

e.g. for 93.2 MHz: http://localhost:2346/93200000

To use this tool in KODI simply create a *.strm file e.g. "FM\_93_2.strm"
 
    http://localhost:2346/93200000

JSON-RPC API
--------------
rtl_fm_streamer comes with a [JSON-RPC](https://en.wikipedia.org/wiki/JSON-RPC) 1.0 API. It is listening at port 2345 but you can specify the port with the parameter "-j".

    $ ./rtl_fm_streamer -j 1234
    
**Provided methods**
| Method | Parameters | Return | Description
|---|---|---|---|
| SetFrequency | Frequency in Hz | Frequency in Hz | Tunes to a given frequency |
| GetPowerLevel | None  |  Power level in DBFS | Returns the current power level in DBFS |

**Example Set Frequency**
client  --> rtl_fm_streamer

    {"method": "SetFrequency", "params": [93200000]}
    
rtl_fm_streamer  --> client
     
    {"result": [93200000]}

Performance
--------------
Mono: Should run on many small devices. e.g. a Raspberry Pi 1.
Stereo: Needs a lot of more CPU power compared to mono (tested on a Raspberry Pi 2).
On modern PCs (x86, x64) mono and stereo decoding should be possible easily (tested with an Intel CORE i7 and an Intel CORE 2 Duo)

Limitations
--------------
- Server accepts only one client

Known Problems
--------------
- Occasional segmentation faults after disconnect of a client

Building
-------
To compile rtl_fm_streamer just do the following steps (install git, cmake and libev first).

    $ git clone https://github.com/AlbrechtL/rtl_fm_streamer.git
    $ cd rtl_fm_streamer/
    rtl_fm_streamer$ mkdir build
    rtl_fm_streamer$ cd build
    rtl_fm_streame/build$ cmake ../
    rtl_fm_streame/build$ make


Support
-------
OpenELEC thread: [http://openelec.tv/forum/126-3rd-party/75537-fm-radio-receiver-for-kodi-for-the-raspberry-pi-1](http://openelec.tv/forum/126-3rd-party/75537-fm-radio-receiver-for-kodi-for-the-raspberry-pi-1)

Write me an e-mail: Albrecht <albrechtloh@gmx.de>

