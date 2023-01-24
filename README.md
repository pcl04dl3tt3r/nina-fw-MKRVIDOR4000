# Arduino NINA-W102 firmware

This firmware uses [Espressif's IDF](https://github.com/espressif/esp-idf)

## Building

1. [Download the ESP32 toolchain](http://esp-idf.readthedocs.io/en/v3.3.1/get-started/index.html#setup-toolchain)
1. Extract it and add it to your `PATH`: `export PATH=$PATH:<path/to/toolchain>/bin`
1. Clone **v3.3.1** of the IDF: `git clone --branch v3.3.1 --recursive https://github.com/espressif/esp-idf.git`
1. Set the `IDF_PATH` environment variable: `export IDF_PATH=<path/to/idf>`
1. Run `make` to build the firmware (in the directory of this read me)
1. Load the `Tools -> SerialNINAPassthrough` example sketch on to the board
1. Use `esptool` to flash the compiled firmware

## Notes
If updating the NINA firmware for an **Arduino UNO WiFi Rev. 2** or **Arduino Nano RP2040** board via [SerialNINAPassthrough](https://github.com/arduino-libraries/WiFiNINA/blob/master/examples/Tools/SerialNINAPassthrough/SerialNINAPassthrough.ino) sketch, then the `esptool` invocation needs to be changed slightly:
```diff
-  --baud 115200 --before default_reset
+  --baud 115200 --before no_reset
```

If updating the NINA firmware for an **Arduino MKR Vidor 4000** board via [SerialNINAPassthrough] and esptool, use the buffered version of the sketch from sameer's patch-4 branch (https://github.com/sameer/WiFiNINA/blob/patch-4/examples/Tools/SerialNINAPassthrough/SerialNINAPassthrough.ino) sketch, and the sketch needs to be changed slightly:
```diff
-  FPGA.digitalWrite(FPGA_NINA_GPIO0, (Serial.dtr() == 1) ? HIGH : LOW);
+  FPGA.digitalWrite(FPGA_NINA_GPIO0, (Serial.dtr() == 0) ? HIGH : LOW);
```

Eample invocation for **Arduino MKR Vidor 4000** board via [SerialNINAPAssthrough]:
esptool.py --no-stub --chip esp32 --port COM4 --baud 119400 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x0 NINA_W102.bin

## Build a new certificate list (based on the Google Android root CA list)
```bash
git clone https://android.googlesource.com/platform/system/ca-certificates
cp nina-fw/tools/nina-fw-create-roots.sh ca-certificates/files
cd ca-certificates/files
./nina-fw-create-roots.sh
cp roots.pem ../../nina-fw/data/roots.pem
```

## Check certificate list against URL list
```bash
cd tools
./sslcheck.sh -c ../data/roots.pem -l url_lists/url_list_moz.com.txt -e
```

## License

Copyright (c) 2018-2019 Arduino SA. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
