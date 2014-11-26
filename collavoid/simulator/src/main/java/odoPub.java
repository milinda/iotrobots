import nav_msgs.Odometry;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Publisher;

/**
 * Created by hjh on 11/25/14.
 */
public class odoPub {

    private odoPubnode odoemetryPublishNode;

    static public class odoPubnode extends AbstractNodeMain {
        private boolean initialized=false;
        //private Publisher<Odometry> odometryPublisher;
        private ConnectedNode node;

        @Override
        public void onStart(ConnectedNode connectedNode) {
            node=connectedNode;
            //use frame ID to identify robot
            //odometryPublisher = connectedNode.newPublisher("Odometry", Odometry._TYPE);
            initialized=true;
        }

        @Override
        public GraphName getDefaultNodeName() {
            return GraphName.of("robot");
        }


        //public Publisher<Odometry> getOdometryPublisher() {
//            return odometryPublisher;
//        }


    }

    public odoPub(String nodeName) {

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic("localhost");
        final NodeMainExecutor executor = DefaultNodeMainExecutor.newDefault();
        nodeConfiguration.setNodeName(nodeName);

        odoemetryPublishNode = new odoPubnode();
        executor.execute(odoemetryPublishNode, nodeConfiguration);

        System.out.println("Initializing Odometry publisher");

        while(!odoemetryPublishNode.initialized) {
            System.out.print(".");
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        System.out.println("Done!");
    }

//    public Publisher<Odometry> getPublisher(){
//
//        if (odoemetryPublishNode.initialized){
//            return odoemetryPublishNode.getOdometryPublisher();
//        }else{
//            System.out.println("Error, not initialized!");
//            return null;
//        }
//    }

    public ConnectedNode getNode(){
        if (odoemetryPublishNode.initialized){
            return odoemetryPublishNode.node;
        }else{
            System.out.println("Error, node not initialized!");
            return null;
        }

    }


}
