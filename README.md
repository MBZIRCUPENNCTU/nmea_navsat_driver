# nmea_navsat_driver

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

## Setting for GPS

### PRECIS-BX305 GNSS RTK BOARD (GPS L1L2/GLONASS G1/BEIDOU B1B3)

Download [Tersus Software Center] (http://www.tersus-gnss.com/pages/document-software)

#### BASESTATION setting
##### factory reset:
```
freset
```

##### To get Basestation fix position from averaging:
1. Use commands
```
fix none
interfacemode com1 automatic automatic on
log com1 gpgga ontime 0.1
saveconfig
```
2. Run position averaging in Tersus GNSS center and copy it [lat, lon, alt].
3. Use commands
```
unlog com1 gpgga
saveconfig
```

##### BASESTATION corrections output
```
fix position lat lon alt
ecutoff bd2 15.0
ecutoff gps 15.0
ecutoff glonass 15.0
interfacemode com2 automatic automatic on
log com2 rtcm1074 ontime 1
log com2 rtcm1084 ontime 1
log com2 rtcm1124 ontime 1
log com2 rtcm1005 ontime 0.05
saveconfig
```

##### BASESTATION additional output for diagnostics
```
interfacemode com1 automatic automatic on
log com1 gpgga ontime 1
log com1 gpgsv ontime 1
saveconfig
```

#### ROVER setting
##### factory reset:
```
freset
```

##### ROVER basic setup
```
fix none      
interfacemode com2 automatic automatic on
interfacemode com1 automatic automatic on
log com1 gpgga ontime 0.05
saveconfig
```

##### For different rate of msgs
* ontime 0.05 - 20Hz
* ontime 0.1 - 10Hz
* ontime 1 - 1 Hz
