# GPS Denied Navigation

A visual positioning system that determines a drone's location by matching a camera image against a georeferenced satellite map, then spoofs that position to ArduPilot over UART — allowing navigation without GPS.
| | |
|---|---|
|<img width="1506" height="991" alt="Original" src="https://github.com/user-attachments/assets/f6145493-a735-4b88-8d54-72c28a14176e" /> | <img width="1647" height="863" alt="Located" src="https://github.com/user-attachments/assets/9c59cedc-0f37-47dd-bd4c-67e9b44d8778" />|

## How it works

A cropped image (simulating a drone camera frame) is matched against a GeoTIFF satellite map using OpenCV feature detection. Once a match is found, the pixel coordinates are converted to latitude/longitude using the geospatial data embedded in the TIF file. That position is then forwarded to a Pixhawk flight controller as NMEA sentences over serial at 1Hz.

## Map source

The reference map is a GeoTIFF exported from Google Earth and processed in QGIS. A US location was chosen due to higher satellite image resolution available. The simulated camera image was captured at ~200m altitude, which made real drone testing difficult at this stage.

## Tested on

- ArduPilot Pixhawk 6C

## Future work

- Replace static crop image with a live camera feed
- Deploy on Raspberry Pi onboard the drone for real-time testing
