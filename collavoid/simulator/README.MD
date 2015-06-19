1. Install J3D library

Copy ./lib/ext/*.jar to $JAVA_HOME/jre/lib/ext/
Copy ./lib/amd64/*.so to $JAVA_HOME/jre/lib/amd64/
Or simply run ./lib/install.sh provided that JAVA_HOME env variable is correctly set

2. Build

Run maven clean install to build the project

3. Run ROS

roscore

4. Run simulator

In target dir run java -jar simulator-collavoid-1.0-SNAPSHOT-jar-with-dependencies.jar -n <number of robot to simulate> \\
-r <pose radius of robots> -ip <ip or host of the machine that run the simulator> -master <ros master url>

Default values for args: n=2, r=2.5, ip="localhost",master="http://localhost:11311"