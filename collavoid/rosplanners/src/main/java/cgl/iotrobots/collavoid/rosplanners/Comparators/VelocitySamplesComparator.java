package cgl.iotrobots.collavoid.rosplanners.Comparators;

import cgl.iotrobots.collavoid.rosplanners.utils.VelocitySample;

import java.util.Comparator;


public class VelocitySamplesComparator implements Comparator<VelocitySample> {

    public int compare(VelocitySample vs1,VelocitySample vs2){
        if(vs1.getDistToPrefVel() < vs2.getDistToPrefVel()){
            return -1;
        }else if (vs1.getDistToPrefVel()>vs2.getDistToPrefVel()){
            return 1;
        }else
            return 0;
    }




}
