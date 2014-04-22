multiwii-mavlink-gc
===================

Connecting Multiwii Flight Controler to QGroundControl with mavlink

Multiwii Serial Protocol decoding exemple in C

Multiwii Serial Protocol log to csv  


Build from source 
===================
	
Windows and Linux Arch :

	$ make


Run
===================

in a terminal navigate to folder src\mavlink :
	
	run "mwgc --help" 
		
		
read data from COM4 and broadcast to groundStation running at ip 192.168.0.13
	
	"mwgc.exe -s COM4 -ip 192.168.0.13"

	
Windows
-------------
	read data from COM4 and broadcast as uav 1 to groundStation running at ip 192.168.0.13
	read data from COM5 and broadcast as uav 2 to groundStation  running at ip 192.168.0.13
	
	"mwgc.exe -s COM4 -id 1 -ip 192.168.0.13"
	"mwgc.exe -s COM6 -id 2 -ip 192.168.0.13"
	
Linux
-------------
	"mwgc -s /dev/ttyUSB1 -ip 127.0.0.1"
