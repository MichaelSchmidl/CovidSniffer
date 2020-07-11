# CovidSniffer

CovidApp Sniffer using an ESP32 based M5-STACK

<img src="2020-07-11 12.47.53.jpg">

As seen, there are four numbers given:

1) ACTIVE is the number beacons or Apps received in the last minute
2) TOTAL is ACTIVE plus any beacons sent during the last minute. Because the Apps change their identity app. every 15 minutes, it can happen that the number of TOTAL is higher than the number of ACTIVE apps.
3) MAX is the maximum number of beacons seen at the same time. Because of the changing identities this number can be up to two timer higher as the actual apps.
4) SUM is the total sum of different app identies seen during runtime

