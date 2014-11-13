package test;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;

public class setParameters extends AbstractNodeMain {
    @Override
    public void onStart(ConnectedNode connectedNode) {
        double acc_lim_th;
        System.out.println(connectedNode.getName().toString());
        ParameterTree params=connectedNode.getParameterTree();
        acc_lim_th=params.getDouble("~/acc_lim_th");
        params.set(connectedNode.getName()+"/max_vel_x",true);
        System.out.println(acc_lim_th);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }
}
