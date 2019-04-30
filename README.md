# ECEN 5823 Bluetooth Mesh Project - Friend Node

<pre>
Link to group report: https://docs.google.com/document/d/1R1eXeu73rT5qerNNjkkjfaU8d4dDdhMGX4tJ_LX3e94/edit

Link to personal report: https://docs.google.com/document/d/1NBM77vrFWzWBoXy0kWyAOh5fD9DEhFWCa_TdBu3d3-8/edit

This code was developed with the help of team members.

Project Status:
1. Configured Friend node (done - working)
2. Established friendship with both LPN (done)
3. Interfaced buzzer and LED (done - working)
4. Interfaced vibration sensor (done - working)
5. Interfaced 2 infrared sensors (done - working)
6. Showing vibration sensor alerts locally (done - working)
7. Publish vibration sensor data (done - working)
8. Publish push button data to control lights at LPN (done - working)
9. Write subscription code for LPN data publish (to be done)
10. Whole mesh working with Friend (done)
11. Added persistent data for display message, alerts and lights led and people count (done)

Models Used:
PB0_STOP_ALERT		- LEVEL model - data 0x01
VIBRATION_ALERT 	- LEVEL model - data 0x0A
LIGHT_CONTROL_ON	- OnOff model - data 0x01
LIGHT_CONTROL_OFF	- OnOff model - data 0x00
GAS_ALERT			- LEVEL model - data 0x0C
FIRE_ALERT 			- LEVEL model - data 0x0D
NOISE_ALERT 		- LEVEL model - data 0x0E
HUMIDITY_ALERT 		- LEVEL model - data 0x0F

</pre>