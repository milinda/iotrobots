Kinect driver
==========================================

SendFrame.java creates an exchange "frames" that compresses every other frame recieved from the Kinect using JZlib, a java implementation of Zlib.
The compressed frames are published as byte[] using RabbitMQ.

RecvFrame.java binds a queue to the "frames" exchange and pops the byte[]. The arrays are decompressed and a color coded depth image is displayed.

Simply run Test/runTest.sh to see it all in action. Upon running the user will be asked to input an ip address for hosting SendFrame.java and a name for the exchange to be created.

Test/runNTest.sh runs the SendFrame and RecvFrame with new compression, these programs are still in development
