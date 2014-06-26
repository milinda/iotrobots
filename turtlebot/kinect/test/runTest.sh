javac -classpath ../lib/rabbitmq-client.jar:../lib/freenect-jna.jar:../lib/jna-4.1.0.jar:../lib/jzlib.jar ../src/main/java/*.java -d .

echo "Type an ip address and press [ENTER]"

read ipAddr

echo "Type a name for the exchange and press [ENTER]"

read exchange_name

x-terminal-emulator -e java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar SendFrame $ipAddr $exchange_name

java -cp .:../lib/commons-io-1.2.jar:../lib/commons-cli-1.1.jar:../lib/rabbitmq-client.jar:../lib/jna-4.1.0.jar:../lib/freenect-jna.jar:../lib/jzlib.jar RecvFrame $exchange_name
