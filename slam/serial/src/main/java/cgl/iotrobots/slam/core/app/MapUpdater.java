package cgl.iotrobots.slam.core.app;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class MapUpdater {
    private Logger LOG = LoggerFactory.getLogger(MapUpdater.class);
    boolean got_map_;

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;

    double xmin;
    double ymin;
    double xmax;
    double ymax;
    double delta;
    double occThresh;
    private OutMap map = new OutMap();

    public MapUpdater(double maxRange_, double maxUrange_, double xmin_, double ymin_, double xmax_, double ymax_, double delta_, double occ_thresh_) {
        this.maxRange_ = maxRange_;
        this.maxUrange_ = maxUrange_;
        this.xmin = xmin_;
        this.ymin = ymin_;
        this.xmax = xmax_;
        this.ymax = ymax_;
        this.delta = delta_;
        this.occThresh = occ_thresh_;
    }

    public void updateMap(LaserScan scan, Particle best, double angles[], DoubleOrientedPoint laserPose) {
        ScanMatcher matcher = new ScanMatcher();
        matcher.setLaserParameters(angles.length, angles, laserPose);

        matcher.setLaserMaxRange(maxRange_);
        matcher.setUsableRange(maxUrange_);
        matcher.setgenerateMap(true);

        if (!got_map_) {
            map.resolution = delta;
            map.origin.x = 0.0;
            map.origin.y = 0.0;
            map.origin.z = 0.0;
            map.originOrientation.x = 0.0;
            map.originOrientation.y = 0.0;
            map.originOrientation.z = 0.0;
            map.originOrientation.w = 1.0;
        }

        DoublePoint center = new DoublePoint((xmin + xmax) / 2.0, (ymin + ymax) / 2.0);
        GMap smap = new GMap(center, xmin, ymin, xmax, ymax, delta);

        map.currentPos.clear();

        LOG.debug("Trajectory tree:");
        synchronized (map.currentPos) {
            for (TNode n = best.node; n != null; n = n.parent) {
                LOG.debug("{} {} {}", n.pose.x, n.pose.y, n.pose.theta);
                if (n.reading == null) {
                    LOG.debug("Reading is NULL");
                    continue;
                }

                IntPoint p = best.map.world2map(n.pose);
                map.currentPos.add(p);

                matcher.invalidateActiveArea();
                double[] readingArray = new double[n.reading.size()];
                System.out.format("best pose: (%f %f %f) reading: (", n.pose.x, n.pose.y, n.pose.theta);
                for (int i = 0; i < n.reading.size(); i++) {
                    readingArray[i] = n.reading.get(i);
                }
                System.out.format(")\n");

                matcher.computeActiveArea(smap, n.pose, readingArray);
                matcher.registerScan(smap, n.pose, readingArray);
            }
        }
        System.out.println();

        // the map may have expanded, so resize ros message as well
        if (map.width != smap.getMapSizeX() || map.height != smap.getMapSizeY()) {

            // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
            // so we must obtain the bounding box in a different way
            DoublePoint wmin = smap.map2world(new IntPoint(0, 0));
            DoublePoint wmax = smap.map2world(new IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));

            ymin = wmin.y;
            xmax = wmax.x;
            ymax = wmax.y;

            LOG.debug("map size is now {} x {} pixels ({},{})-({}, {})", smap.getMapSizeX(), smap.getMapSizeY(),
                    xmin, ymin, xmax, ymax);

            map.width = smap.getMapSizeX();
            map.height = smap.getMapSizeY();
            map.origin.x = xmin;
            map.origin.y = ymin;
            map.resize(map.width * map.height);

            LOG.debug("map origin: (%f, %f)", map.origin.x, map.origin.y);
        }
        int count = 0;
        for (int x = 0; x < smap.getMapSizeX(); x++) {
            for (int y = 0; y < smap.getMapSizeY(); y++) {
                /// @todo Sort out the unknown vs. free vs. obstacle thresholding
                IntPoint p = new IntPoint(x, y);
                PointAccumulator pointAccumulator = (PointAccumulator) smap.cell(p, false);
                double occ = pointAccumulator.doubleValue();
                assert (occ <= 1.0);
                //System.out.println("threshold: "  + occThresh  + " occ: "  + occ);
                if (occ < 0) {
                    map.data[MAP_IDX(map.width, x, y)] = -1;
                } else if (occ > occThresh) {
                    //map.map.data[MAP_IDX(map.map.info.width, x, y)] = (int)round(occ*100.0);
                    map.data[MAP_IDX(map.width, x, y)] = 100;
                    count++;
                } else {
                    map.data[MAP_IDX(map.width, x, y)] = 0;
                }
            }
        }
        System.out.println("count " + count);
        got_map_ = true;
    }

    public static int MAP_IDX(int sx, int i, int j) {
        return ((sx) * (j) + (i));
    }
}
