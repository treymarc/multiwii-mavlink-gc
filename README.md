multiwii-mavlink-gc
===================

Connecting Multiwii Flight Controler to QGroundControl with mavlink

Multiwii Serial Protocol decoding exemple in C

Multiwii Serial Protocol log to csv  


Build from source 
===================
	
Windows and Linux, unzip and type:

	$ make
	
build with debug messages, show messages:

	$ make VERBOSE=3



Run
===================

in a terminal navigate to folder src\mavlink :
	
	run "mwgc --help" 
		
		
read data from COM4 and broadcast to groundStation running at ip 192.168.0.13
	
	"mwgc.exe -s COM4 -ip 192.168.0.13"


you can change the refresh rate or the baudrate

	"mwgc -s /dev/ttyUSB0 -hertz 50 -baudrate 57600"
	
Windows
-------------
	read data from COM4 and broadcast as uav 1 to groundStation running at ip 192.168.0.13
	read data from COM5 and broadcast as uav 2 to groundStation  running at ip 192.168.0.13
	
	"mwgc.exe -s COM4 -id 1 -ip 192.168.0.13"
	"mwgc.exe -s COM6 -id 2 -ip 192.168.0.13"
	
Linux
-------------
	"mwgc -s /dev/ttyUSB1 -ip 127.0.0.1"
	

You can set the serial port option by hand :

    "stty -F /dev/ttyUSB0  cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts "
     
