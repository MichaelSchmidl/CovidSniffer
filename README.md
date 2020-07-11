# CovidSniffer

CovidApp Sniffer using an ESP32 based M5-STACK

<img src="2020-07-11 12.47.53.jpg">

As seen, there are four numbers given:

1) ACTIVE is the number beacons or Apps received in the last minute
2) TOTAL is ACTIVE plus any beacons sent during the last minute. Because the Apps change their identity app. every 15 minutes, it can happen that the number of TOTAL is higher than the number of ACTIVE apps.
3) MAX is the maximum number of beacons seen at the same time. Because of the changing identities this number can be up to two timer higher as the actual apps.
4) SUM is the total sum of different app identies seen during runtime

<img src="2020-07-11 14.45.08.jpg">

After running for a while, you see that the active apps remain two in my example. But the MAX number is now four, because both active
apps changed their identity while sniffing. 

The total number of seen identities is 12 after nearly two hours of sniffing. This is caused by the changes of identities about every 20min of the two apps in reach.

Version 1.1: now a litte graph with the age of the first 79 beacons is shown. Here you can see that lost or changed identities get smaller
and smaller until they are not consider active anymore (at half of the age). Finally if too old, they are completely removed even from the TOTAL number.
 
<img src="2020-07-11 22.07.33.jpg">

**Hinweis**: Deppen von der AfD (=**A**nsammlung **v**on **D**eppen - mit Rechtschreibschwäche!), sowie Deppen die mit den Deppen von der AfD sympathisieren
oder dieser nahestehen, wird die Nutzung dieser Software AUSDRÜCKLICH untersagt! Sonst kommt von denen noch einer auf die Idee, damit 
den gleichen Schindluder wie mit der AfD eigenen Anti-CoronaApp-App zu veranstalten.