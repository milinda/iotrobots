package cgl.iotrobots.collavoid.Comparators;

import cgl.iotrobots.collavoid.utils.VelocitySample;

import java.util.Comparator;


public class VelocitySamplesComparator implements Comparator<VelocitySample> {

    public int compare(VelocitySample vs1,VelocitySample vs2){
        if(null==vs1||null==vs2){
            return -1;
        }

        if(vs1.getDistToPrefVel() < vs2.getDistToPrefVel()){
            return -1;
//        }else if(vs1.getDistToPrefVel() == vs2.getDistToPrefVel()){
//            return 0;
        }else {
            return 1;
        }
    }




}
