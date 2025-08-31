# Instructable

Een instructable is een stappenplan - zonder verdere uitleg - hoe je vertrekkend van de bill of materials en gebruik makend van de technische tekeningen de robot kan nabouwen. Ook de nodige stappen om de microcontroller te compileren en te uploaden staan beschreven.  

Korte gebruiksaanwijzing – ESP32 Line-Follower 
1) Wat heb je nodig
•	ESP32 (WROOM)
•	DRV8833 motor driver
•	2 DC-motoren met wielen 
•	QTR-8A lijnsensor (8 × analoog)
•	Batterijen
•	Jumping wires
•	Smartphone met Bluetooth-terminal app 
2) Aansluiten (beknopt)
•	DRV8833 ↔ motoren
o	Motor A op AOUT1/AOUT2, motor B op BOUT1/BOUT2
•	DRV8833 ↔ ESP32
o	AIN1 = 22, AIN2 = 21 (motor links)
o	BIN1 = 19, BIN2 = 18 (motor rechts)
o	nSLEEP = 23 → HOOG (driver aan)
o	GND ESP32 ↔ GND DRV8833 (gedeelde massa)
o	VM = motorvoeding (niet de 3V3 van de ESP32!)
•	QTR-8A ↔ ESP32
OUT-pinnen naar 34, 35, 32, 33, 25, 26, 14, 39 (allemaal inputs).
Voed de QTR bij voorkeur met 3.3 V als je direct op de ESP32 meet.
•	Zet de sensor ~5–8 mm boven de ondergrond; gebruik matte, donkere lijn.
3) Firmware flashen
•	Open de Arduino IDE → Board: ESP32 Dev Module +  juiste COM-poort selecteren.
•	Upload de arduino code 
•	In de BT-app zie je het apparaat “ESP32-LineBot”.
4) Eerste gebruik
1.	Voeding aan (motorvoeding + ESP32 USB).
2.	De robot kalibreert kort; beweeg de robot 2 s over wit én zwart als je c intypt
3.	Open je BT-app, verbind met ESP32-LineBot.
4.	Commando 1 → éénmalig de 8 RAW sensormetingen (zwart ≈ 4095, wit ≈ 2000).
5.	Commando s → starten/stoppen.
5) Bediening via Bluetooth (commando’s)
•	s – start/stop rijden
•	c – (her)kalibreren (beweeg over wit/zwart tijdens ~2 s)
•	1 – toon éénmalig 8 RAW sensorwaarden
•	r – toon huidige PID en snelheden
•	p=1.2 / i=0.00 / d=6.0 – stel PID in
•	b=160 – basis¬snelheid (0..255)
•	b+ / b- – basis¬snelheid +10 / −10
•	max=220 – maximale dutycap (0..255) om bochten te temperen
•	slow / fast – snelheids-presets
•	stop – noodstop (running=false)
6) Afstellen (zodat hij de lijn mooi volgt)
1.	Kalibreren: stuur c en beweeg over lijn + achtergrond.
2.	P verhogen tot hij nét begint te slingeren → dan iets terug.
3.	D verhogen om te dempen (5 → 7 → 8).
4.	I heel klein (0.00–0.02) alleen als hij structureel naast de lijn blijft.
5.	Bochten te scherp? Verlaag p of max, of verhoog d.
6.	Te traag op rechte stukken? b+ of b=… wat hoger.
7) Richting en mechaniek
•	Draait hij “verkeerd om”? Wissel motorpolen van de betreffende motor.
•	Zorg dat beide wielen vrij lopen; rijhoogte sensor 5–8 mm.
•	Caster/geleider soepel; accu stevig vast, draden kort.



