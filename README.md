# MySensors-Garage

Multisensor, actively switching garage LED-light on if motion+dark
- 2 x Motion
- LightLux
- Pressure+Temp (BMP180)
- Relay
- configurable on-time and darkness-level

Based on MySensors 2.0.0-dev


Er funktioniert zwar weitgehend, aber leider muß ich vermutlich doch 
- zwei interupt-handler-Routinen definieren (jeweils eine für die beiden SR501, analog dem counter-Beispiel)
- das sleep() ganz sein lassen, sonst ist er "nur" über einen Restart konfigurierbar (hängt eh' an einem Netzteil)
- dementsprechend den Code für den BMP dahingehend anpassen, dass er nur jede Minute abgefragt wird (nicht wie jetzt bei jedem Aufwachen).
