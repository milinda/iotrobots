Storm options
-------------
./bin/storm jar ~/projects/iotrobots/collectives/target/collectives-1.0-SNAPSHOT-jar-with-dependencies.jar edu.iu.cs.storm.collectives.app.BroadCastTopology  -name bcast_processor -ds_mode 0 -p 8
./bin/storm kill bcast_processor -w 1

DataGenerator options
---------------------
"amqp://149.165.158.215:5672" 2000