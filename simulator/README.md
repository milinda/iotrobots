Setting up Simbard based Simulator
---------------------------------

IntelliJIdea

Add the full path of simulator/native/lib/amd64 to you LD_LIBRAY_PATH
In the module dependencies add the jars in simulator/native/lib/ext as dependencies 
Now run the SimbardExample from the IntelliJIdea

Running FileBased Simulator
---------------------------

There are two file based simulators. The FileBasedSimulator runs the serial/thread based parallel version of the algorithm.
The FileBasedDistributedSimulator runs the Storm version of the algorithm. The parameters are hard coded in those at the moment. 
So you need to compile them with different broker URLS. 

First you need to build the simulator module using maven. 
 
mvn clean install

Then you need to copy the out.txt to the place where you run the following commands.

java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedSimulator 4

java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedDistributedSimulator "amqp://149.165.159.3:5672"
java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedDistributedSimulator "amqp://localhost:5672" simbard_0.txt simbard_60_20

java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedDistributedSimulator "amqp://localhost:5672" simulator/data/aces.txt aces_60_20 false true
"amqp://149.165.159.12:5672" simulator/data/aces.txt aces_60_20 false true

java -Xmx6G -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedSimulator true simulator/data/aces.txt 90 false 4 false

SIMBARD_TEST
------------
./bin/storm jar ~/projects/iotrobots/slam/streaming/target/iotrobots-slam-streaming-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.streaming.SLAMTopology -name slam_processor -ds_mode 0 -p 4 -pt 20 -i
./bin/storm kill slam_processor -w 1

./bin/iotcloud jar repository/sensors/iotrobots-slam-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.sensor.SlamSensor -s local -sim -url "amqp://10.39.1.28:5672"


 java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedDistributedSimulator "amqp://10.39.1.28:5672" data/simbard_1.txt 20_8 true false 1500