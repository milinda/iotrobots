#!/bin/sh
# compiles all files in Kinect folder and runs with given parameters

javac -classpath ../lib/rabbitmq-client.jar:../lib/freenect-jna.jar:../lib/jna-4.1.0.jar:../lib/jzlib.jar:../../lib/snappy-java-1.1.1.jar ../src/main/java/*.java -d .

# run SendFrame.java in a new terminal 
#(x-terminal... command doesn't work on all machines)
java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar:../lib/snappy-java-1.1.1.jar SendFrame_new $ipAddr $@





