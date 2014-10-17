package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.utils.OrientedPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class GridSlamProcessor {
    private List<TNode> TNodeVector = new ArrayList<TNode>();
    private BlockingDeque<TNode> TNodeDeque = new LinkedBlockingDeque<TNode>();
    private List<Particle> ParticleVector = new ArrayList<Particle>();


    double srr;
    double srt;
    double str;
    double stt;
    double minimumScore;
}
