# Tankniveau

Dieses Programm ist für die Ueberwachung der Heizung. Angeschlossen sind ein Ultraschall Distanz-
sensor um den Füllstand des Oeltanks zu messen und 4 DS1820 Temperatursensoren. Diese messen
die Temperatur im Speichertank sowie Vor- und Rücklauftemperatur.
Die Messwerte werden via MQTT versendet.



<img src="https://github.com/HB9UQS/Tankniveau/blob/master/doc/tankniveau_Steckplatine.png" width="200">



------
## Components

- Board: ESP32 with OLED: Heltec WiFi LoRa 32(V1)
- Sensor: [SEN0313](https://wiki.dfrobot.com/A01NYUB%20Waterproof%20Ultrasonic%20Sensor%20SKU:%20SEN0313)
- Temperature Sensor DS1820
  

------
## Installation instructions

- Copy config.h.default to config.h and adapt to your needs
- Note that the ID Values of the Sensors were adapted before. Therefore it will not work with factory default sensors.  


------

## Additional Ideas

none

------

## History

##### 1.0 Initial Version - (2023-11-2609)


