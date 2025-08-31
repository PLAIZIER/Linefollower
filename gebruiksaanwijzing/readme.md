# Gebruiksaanwijzing

### opladen / vervangen batterijen
uitleg over het opladen of vervangen van de batterijen
Er zijn 2 batterijen die onderaan de robot zijn bevestigd en makkelijk kunnen verwisseld worden of uit de robot gehaald worden om in de lader te steken.

### draadloze communicatie
#### verbinding maken
uitleg over het verbinden van de robot met laptop / smartphone
De esp heeft een ingebouwde bluetooth module, als we in de arduino code een simpele lijn schrijven zal de ESP verbinding proberen maken via bluetooth. Dan kunnen we in onze smartphone een app installeren die zal communiceren met de ESP.

#### commando's
debug [on/off]  
start  s
stop  s
set cycle [µs]  
set power [0..255]  b 
set diff [0..1]  d
set kp [0..]  p
set ki [0..]  i
set kd [0..]  d
calibrate black  c 
calibrate white  c

### kalibratie
uitleg kalibratie  
Tijdens de kalibratie dient de robot de witte en zwarte lijnen te zien van hier uit kan hij dan de sensorwaaren lezen en kijken wat het verschil is tussen wit & zwart. door het commanda "c" in te voeren zal de robot dit doen

### settings
De robot rijdt stabiel met volgende parameters: 
b op max( 255), p op 15.000, i op 0.000 en d op 7.000

### start/stop button
uitleg locatie + werking start/stop button
een start stop button zit in de smartphone geintregreerd. door het commanda s in te voeren zal de robot stoppen of starten
