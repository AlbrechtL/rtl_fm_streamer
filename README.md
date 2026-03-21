RTL SDR FM Streamer
===================
Turns a Realtek RTL2832U based swradio device into an FM radio stereo receiver.

WARNING: The swradio implementation is AI generated and it seems that the gain control is missing. Most likely you will hear only noise.

Description
-----------
RTL SDR FM Streamer is a small tool to stream FM stereo radio by using the Linux kernel swradio API to a client such as Kodi, VLC or mplayer.

The DVB-T dongle has to be based on the Realtek RTL2832U and exposed by the kernel as a swradio device such as `/dev/swradio0`.

Usage
-----
Default port: 2346

    $ ./rtl_fm_streamer

By default the streamer opens `/dev/swradio0`. Use `-d /dev/swradioN` to select a different kernel SDR device.

**Docker Image**

Thanks to mrbluebrett you can also use a Docker image
https://hub.docker.com/r/mrbluebrett/rtl_fm_streamer

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

To use this tool in KODI simply create a *.strm file e.g. "FM\_93_7.strm"
 
  http://localhost:12345/93700000/1

JSON-RPC API
--------------
rtl_fm_streamer comes with a [JSON-RPC](https://en.wikipedia.org/wiki/JSON-RPC) 1.0 API. It is listening at port 2345 but you can specify the port with the parameter "-j".

    $ ./rtl_fm_streamer -j 1234
    
**Provided methods**

Method | Parameters | Return | Description
------ | ---------- | ------ | -----------
SetFrequency | Frequency in Hz | Frequency in Hz | Tunes to a given frequency
GetPowerLevel | None  |  Power level in DBFS | Returns the current power level in DBFS

**Example Set Frequency**
client  --> rtl_fm_streamer

    {"method": "SetFrequency", "params": [93700000]}
    
rtl_fm_streamer  --> client
     
    {"result": [93700000]}

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
- Hopefully none ;-)

Building
-------
The current maintained build uses CMake and the Linux V4L2 SDR headers.

  $ sudo apt-get install build-essential cmake
    $ git clone https://github.com/AlbrechtL/rtl_fm_streamer.git
    $ cd rtl_fm_streamer/
    rtl_fm_streamer$ mkdir build
    rtl_fm_streamer$ cd build
    rtl_fm_streamer/build$ cmake ../
    rtl_fm_streamer/build$ make

At runtime the kernel must provide a swradio node for the RTL2832U, for example via `rtl2832_sdr`.

Useful diagnostics:

  $ v4l2-ctl --all --device=/dev/swradio0
  $ v4l2-ctl --list-formats --device=/dev/swradio0

Similar Projects
----------------
- FM Radio receiver based upon RTL-SDR as pvr addon for KODI
  - http://esmasol.de/open-source/kodi-add-on-s/fm-radio-receiver/
  - https://github.com/xbmc/xbmc/pull/6174
  - https://github.com/AlwinEsch/pvr.rtl.radiofm
- rtl_fm
  - This tool is the base of rtl_fm_streamer
  - http://sdr.osmocom.org/trac/wiki/rtl-sdr
- sdr-j-fmreceiver
  - http://www.sdr-j.tk/index.html
- GPRX
  - http://gqrx.dk

