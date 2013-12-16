multiwii-mavlink-gc
===================

Connecting Multiwii Flight Controler to QGroundControl  with mavlink

Multiwii serial protocol decoding exemple in C

Requirements :
===================

	QgroundControl : http://qgroundcontrol.org/downloads

		-> you can run the demo file
	
	
	Multiwii : http://www.multiwii.com/
	
		-> you can connect to a working mwc with MultwiiConf_2.0 




Build from source :
===================

	
Windows only :
	
	uncomment the line "WINBUILD := true" in the Makefile 
	
all Arch :

	$ make



Run :
===================


in a terminal :
	
	run "mwgc --help" 
		
		
exemple :
	
	read data from COM4 and broadcast to groundStation running at ip 192.168.0.13
	
	"mwgc.exe -s COM4 -ip 192.168.0.13"

	
WIN
	
	read data from COM4 and broadcast as uav 1 to groundStation running at ip 192.168.0.13
	read data from COM5 and broadcast as uav 2 to groundStation  running at ip 192.168.0.13
	
	"mwgc.exe -s COM4 -id 1 -ip 192.168.0.13"
	"mwgc.exe -s COM6 -id 2 -ip 192.168.0.13"
	
Linux
	
	"mwgc -s /dev/ttyUSB1 -ip 127.0.0.1"
