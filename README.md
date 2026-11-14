# ESP-REACHER
### 2.4Ghz Wi-Fi, Bluetooth and BLE jamming tool
### What it is
ESP-REACHER is a 2.4Ghz jamming tool that works on Wi-Fi, Bluetooth, BLE, Zigbee, USB wireless and more.
> **Name inspired by Reacher (Amazon prime TV series)**
<!-- 
### What's different from the industry standard
NRF24LO1-based jamming tools presently being used in the industry rely on the ```startConstCarrier()``` function along with channel hopping to create noise. ESP-REACHER generates payloads of multi-spectrum noise. ### Why it's better than the industry standard
* Modern devices have filters to block out unmodulated sine waves, making ```startConstCarrier()``` less effective
* ```startConstCarrier()``` occupies only 1Mhz, unlike payloads, which take up an extra 0.5Mhz above and below the targeted channel
* Receiver can't safely ignore noise without corrupting it's own signal
-->
### Hardware used:
* NRF24LO1 X3
* ESP32 C3 SuperMINI X1
* 0.96" OLED monochrome display X1
* Button X1
