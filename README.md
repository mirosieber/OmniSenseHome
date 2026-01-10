# OmniSensHome
Software for a multi sensor board based on ESP32 C6 communicating via Zigbee.

in sdkconfig muss CONFIG_FREERTOS_HZ von 100 auf 1000 Hz erhöht werden! sonst compile fehler

um neues konfig file zu laden zum Beispiel um Fensterkontakt hinzuzufügen, kompletten speicher überschreiben und dann config und code laden, um sicherzustellen das zigbee neu verbinden und konfigurieren muss.


wenn das Programm nicht mehr kompilieren will: 


idf.py fullclean (wenn das nicht geht von hand den managed_components ordner löschen und idf.py clean ausführen)
idf.py build

