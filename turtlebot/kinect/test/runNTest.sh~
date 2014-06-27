#!/bin/sh
# compiles all files in Kinect folder and runs with given parameters

javac -classpath ../lib/rabbitmq-client.jar:../lib/freenect-jna.jar:../lib/jna-4.1.0.jar:../lib/jzlib.jar ../src/main/java/*.java -d .

echo "Select an ip address"
#press [1] for 156.56.93.102
#   or [2] for localhost
#   or enter a new ip"
read ipAddr
#if [ "$ipAddr" == "1" ]; then
#	ipAddr="156.56.93.102"
#elif [ "$ipAddr" == "2" ]; then
#	ipAddr="localhost"
#fi

echo "Type a name for the exchange and press [ENTER]"
read exchange_name

#echo "Display depth data? [y/n]"
#read answer

# run SendFrame.java in a new terminal 
#(x-terminal... command doesn't work on all machines)
x-terminal-emulator -e java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar SendFrame_new $ipAddr $exchange_name

#if [ "$answer" == "y" ]; then
	java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar RecvFrame_new $exchange_name
#fi



