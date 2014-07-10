#!/bin/sh
# compiles all files in Kinect folder and runs with given parameters

javac -classpath ../lib/rabbitmq-client.jar:../lib/freenect-jna.jar:../lib/jna-4.1.0.jar:../lib/jzlib.jar ../src/main/java/*.java -d .

echo "Select an ip address"
read ipAddr

# run SendFrame.java in a new terminal 
#(x-terminal... command doesn't work on all machines)
x-terminal-emulator -e java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar SendFrame_new $ipAddr 

java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar RecvFrame_new 




