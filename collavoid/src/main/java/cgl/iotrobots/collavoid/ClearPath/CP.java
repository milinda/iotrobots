package cgl.iotrobots.collavoid.ClearPath;

import cgl.iotrobots.collavoid.utils.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Logger;

import static cgl.iotrobots.collavoid.utils.utils.leftOf;
import static cgl.iotrobots.collavoid.utils.utils.rightOf;

public class CP {

    public static final int RAYRAY= 0;
    public static final int RAYSEGMENT=1;
    public static final int SEGMENTSEGMENT= 2;
    public static final int SEGMENTLINE= 3;
    public static final int RAYLINE= 4;
    public static final int LINELINE= 5;

    public static final int  HRVOS= 0;
    public static final int  RVOS= 1;
    public static final int  VOS= 2;


    public static Vector2 calculateClearpathVelocity(List<VelocitySample> samples, final List<VO> truncated_vos, final List<Line> additional_constraints, final Vector2 pref_vel, double max_speed, boolean use_truncation) {

        if (!isWithinAdditionalConstraints(additional_constraints, pref_vel)) {
            for(Iterator<Line> line=additional_constraints.iterator();line.hasNext();){
                VelocitySample pref_vel_sample=new VelocitySample();
                pref_vel_sample.setVelocity(intersectTwoLines(line.next().getPoint(), line.next().getDir(), pref_vel, new Vector2(line.next().getDir().getY(), -line.next().getDir().getY())));
                pref_vel_sample.setDistToPrefVel(Vector2.absSqr(Vector2.minus(pref_vel, pref_vel_sample.getVelocity())));
                samples.add(pref_vel_sample);
            }
        }
        else {
            VelocitySample pref_vel_sample=new VelocitySample();
            pref_vel_sample.setVelocity(pref_vel);
            pref_vel_sample.setDistToPrefVel(0);
            samples.add(pref_vel_sample);
        }
        VelocitySample null_vel_sample=new VelocitySample();
        null_vel_sample.setVelocity(new Vector2(0,0));
        null_vel_sample.setDistToPrefVel(Vector2.absSqr(pref_vel));
        samples.add(null_vel_sample);

        for(Iterator<Line> line=additional_constraints.iterator();line.hasNext();){
            for(Iterator<Line> line2=additional_constraints.iterator();line2.hasNext();){
                   addRayVelocitySamples(samples, pref_vel, line.next().getPoint(), line.next().getDir(),line2.next().getPoint(), line2.next().getDir(), max_speed, LINELINE);
            }
        }

        for (int i= 0 ; i<  truncated_vos.size(); i++){
            if (isInsideVO(truncated_vos.get(i), pref_vel, use_truncation)){

                VelocitySample leg_projection=new VelocitySample();
                if (leftOf(truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRelativePosition(), pref_vel)){ //left of centerline, project on left leg
                    leg_projection.setVelocity(intersectTwoLines(truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(),
                            pref_vel,new Vector2(truncated_vos.get(i).getLeftLegDir().getY(), -truncated_vos.get(i).getLeftLegDir().getX())));
                }
                else { //project on right leg
                    leg_projection.setVelocity(intersectTwoLines(truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(),
                            pref_vel, new Vector2(truncated_vos.get(i).getRightLegDir().getY(), -truncated_vos.get(i).getRightLegDir().getX())));
                }

                //if(absSqr(leg_projection.velocity) < max_speed) { //only add if below max_speed
                leg_projection.setDistToPrefVel(Vector2.absSqr(Vector2.minus(pref_vel,leg_projection.getVelocity())));
                samples.add(leg_projection);
                //}

                if (use_truncation) {
                    addRayVelocitySamples(samples, pref_vel, pref_vel, Vector2.negative(truncated_vos.get(i).getRelativePosition()),
                            truncated_vos.get(i).getTruncLeft(), Vector2.minus(truncated_vos.get(i).getTruncRight() , truncated_vos.get(i).getTruncLeft()), max_speed, RAYSEGMENT);
                }
            }
        }


        for (int i= 0 ; i<  truncated_vos.size(); i++){
            for (int j = 0; j< additional_constraints.size(); j++){
                if(!use_truncation) {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                }
                else {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), Vector2.minus(truncated_vos.get(i).getTruncRight() , truncated_vos.get(i).getTruncLeft()),
                            additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, SEGMENTLINE);
                }
            }

            for (int j = i+1; j <  truncated_vos.size(); j++){

                if (!use_truncation) {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);

                }
                else {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);


                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getTruncLeft(),
                            Vector2.minus(truncated_vos.get(j).getTruncRight() , truncated_vos.get(j).getTruncLeft()), max_speed, RAYSEGMENT); //left trunc
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getLeftLegDir(), truncated_vos.get(i).getTruncLeft(),
                            Vector2.minus(truncated_vos.get(i).getTruncRight() , truncated_vos.get(i).getTruncLeft()), max_speed, RAYSEGMENT); //trunc left

                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getTruncLeft(),
                            Vector2.minus(truncated_vos.get(j).getTruncRight() , truncated_vos.get(j).getTruncLeft()), max_speed, RAYSEGMENT); //right trunc
                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getRightLegDir(), truncated_vos.get(i).getTruncLeft(),
                            Vector2.minus(truncated_vos.get(i).getTruncRight() , truncated_vos.get(i).getTruncLeft()), max_speed, RAYSEGMENT); //trunc right

                    addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), Vector2.minus(truncated_vos.get(i).getTruncRight(),truncated_vos.get(i).getTruncLeft()),
                            truncated_vos.get(j).getTruncLeft(), Vector2.minus(truncated_vos.get(j).getTruncRight() , truncated_vos.get(j).getTruncLeft()), max_speed, SEGMENTSEGMENT); //trunc trunc


                }

            }
        }


        //    ROS_ERROR("projection list length  = %d", samples.size());

        Collections.sort(samples,new VelocitySamplesComparator());

        Vector2 new_vel=new Vector2();

        boolean valid;
        boolean foundOutside = false;
        boolean outside;
        int optimal = -1;

        for (int i = 0; i <  samples.size(); i++) {
            outside = true;
            valid = true;
            if (!isWithinAdditionalConstraints(additional_constraints, samples.get(i).getVelocity()) ) {
                outside = false;
            }

            for (int j = 0; j <  truncated_vos.size(); j++) {
                if (isInsideVO(truncated_vos.get(j), samples.get(i).getVelocity(), use_truncation)) {
                    valid = false;
                    if (j> optimal) {
                        optimal = j;
                        new_vel = samples.get(i).getVelocity();
                    }
                    break;
                }
            }
            if (valid && outside){
                return samples.get(i).getVelocity();
            }
            if (valid && !outside && !foundOutside) {
                optimal = truncated_vos.size();
                new_vel = samples.get(i).getVelocity();
                foundOutside = true;
            }

        }
        //    ROS_INFO("selected j %d, of size %d", optimal, (int) truncated_vos.size());


        return new_vel;
    }

    static boolean isWithinAdditionalConstraints(final List<Line> additional_constraints, final Vector2 point) {
        for(Iterator<Line> line=additional_constraints.iterator();line.hasNext();){
           if (rightOf(line.next().getPoint(), line.next().getDir(), point) ) {
                return false;
            }
        }
        return true;
    }


    static boolean isInsideVO(VO vo, Vector2 point, boolean use_truncation) {
        boolean trunc = leftOf(vo.getTruncLeft(), Vector2.minus(vo.getTruncRight(),vo.getTruncLeft()), point);
        if (Vector2.abs(Vector2.minus(vo.getTruncLineCenter(), vo.getTruncRight())) < EPSILON.EPSILON)
            trunc = true;
        return rightOf(vo.getPoint(),vo.getLeftLegDir(), point) && leftOf(vo.getPoint(), vo.getRightLegDir(), point) && (!use_truncation || trunc);
    }

    static void addRayVelocitySamples(List<VelocitySample> samples, final Vector2 pref_vel, Vector2 point1, Vector2 dir1, Vector2 point2, Vector2 dir2, double max_speed, int TYPE) {
        double r, s;

        double x1, x2, x3,x4, y1, y2,y3,y4;
        x1 = point1.getX();
        y1 = point1.getY();
        x2 = x1 + dir1.getX();
        y2 = y1 + dir1.getY();
        x3 = point2.getX();
        y3 = point2.getY();
        x4 = x3 + dir2.getX();
        y4 = y3 + dir2.getY();

        double det = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));

        if (det == 0.0) {
            //ROS_WARN("No Intersection found");
            return;
        }
        if (det != 0){
            r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / det;
            s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / det;

            if ( (TYPE == LINELINE) || (TYPE == RAYLINE && r>=0 )  || (TYPE == SEGMENTLINE && r>= 0 && r <= 1) || (TYPE == RAYRAY && r>= 0 && s >= 0) ||
                    (TYPE == RAYSEGMENT && r>=0 && s >= 0 && s <=1) || (TYPE==SEGMENTSEGMENT && r>=0 && s>=0 && r<=1 && s<=1)  ) {


                VelocitySample intersection_point=new VelocitySample();
                intersection_point.setVelocity(new Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1)));
                intersection_point.setDistToPrefVel(Vector2.absSqr(Vector2.minus(pref_vel, intersection_point.getVelocity())));
                if (Vector2.absSqr(intersection_point.getVelocity()) < Math.pow(1.2 * max_speed, 2)){
                    //ROS_ERROR("adding VelocitySample");
                    samples.add(intersection_point);
                    //ROS_ERROR("size of VelocitySamples %d", (int) samples->size());

                }
            }
        }
    }


    public static List<Vector2> minkowskiSum(final List<Vector2> polygon1, final List<Vector2> polygon2){
        List<Vector2> result=new ArrayList<Vector2>();
        List< ConvexHullPoint > convex_hull=new ArrayList<ConvexHullPoint>();

        for (int i = 0; i <  polygon1.size(); i++) {
            for (int j = 0; j <  polygon2.size(); j++) {
                ConvexHullPoint p=new ConvexHullPoint();
                p.setPoint(Vector2.plus(polygon1.get(i),polygon2.get(j)));
                convex_hull.add(p);
            }

        }
        convex_hull = convexHull(convex_hull,false);
        for (int i = 0; i< convex_hull.size(); i++) {
            result.add(convex_hull.get(i).getPoint());
        }
        return result;

    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    //Wikipedia Monotone chain...
    private static List<ConvexHullPoint > convexHull(List<ConvexHullPoint > P, boolean sorted)
    {
        int n = P.size(), k = 0;
        List<ConvexHullPoint> result=new ArrayList<ConvexHullPoint>(2*n);

        // Sort points lexicographically
        if (!sorted)
        Collections.sort(P,new VectorsLexigraphicComparator());


        //    ROS_WARN("points length %d", (int)P.size());

        // Build lower hull
        for (int i = 0; i < n; i++) {
            while (k >= 2 && cross(result.get(k-2), result.get(k-1), P.get(i)) <= 0) k--;
            result.add(k++,P.get(i));
        }

        // Build upper hull
        for (int i = n-2, t = k+1; i >= 0; i--) {
            while (k >= t && cross(result.get(k-2), result.get(k-1), P.get(i)) <= 0) k--;
            result.add(k++,P.get(i));
        }
        //result.resize(k);

        return result;
    }

    static double cross(final ConvexHullPoint o, final ConvexHullPoint A, final ConvexHullPoint B){
        return (A.getPoint().getX()- o.getPoint().getX()) * (B.getPoint().getY() - o.getPoint().getY())
                -(A.getPoint().getY() - o.getPoint().getY()) * (B.getPoint().getX() - o.getPoint().getX());
    }

    public static VO createObstacleVO(Vector2 position1, double radius1, final List<Vector2> footprint1, Vector2 obst1, Vector2 obst2){
        VO result=new VO();

        Vector2 position_obst = Vector2.mul(Vector2.plus(obst1,obst2),0.5);

        List<Vector2> obst=new ArrayList<Vector2>();
        obst.add(Vector2.minus(obst1, position_obst));
        obst.add(Vector2.minus(obst2,position_obst));

        List<Vector2> mink_sum = minkowskiSum(footprint1, obst);

        Vector2 min_left, min_right;
        double min_ang =  0.0;
        double max_ang = 0.0;
        Vector2 rel_position = Vector2.minus(position_obst,position1);

        Vector2 rel_position_normal = Vector2.normal(rel_position);
        double min_dist = Vector2.abs(rel_position);

        for (int i = 0; i<  mink_sum.size(); i++){
            double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position,mink_sum.get(i)));
            if (rightOf(new Vector2(0.0,0.0), rel_position, Vector2.plus(rel_position,mink_sum.get(i)))) {
                if (-angle < min_ang) {
                    min_right = Vector2.plus(rel_position,mink_sum.get(i));
                    min_ang = -angle;
                }
            }
            else {
                if (angle > max_ang) {
                    min_left = Vector2.plus(rel_position,mink_sum.get(i));
                    max_ang = angle;
                }
            }
            Vector2 project_on_rel_position = intersectTwoLines(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position,mink_sum.get(i)), rel_position_normal);
            double dist = Vector2.abs(project_on_rel_position);
            if (Vector2.dotProduct(project_on_rel_position,rel_position) < -EPSILON.EPSILON){
                //	ROS_ERROR("Collision?");
                dist = -dist;

            }

            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist < 0) {
            result.setLeftLegDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));
            result.setRightLegDir(Vector2.negative(result.getLeftLegDir()));

            result.setPoint(Vector2.minus(rel_position,Vector2.mul(Vector2.normal(result.getLeftLegDir()),1.5 * radius1))) ;
            result.setTruncLeft(result.getPoint());
            result.setTruncRight(result.getPoint());
            return result;

        }

        double ang_rel = Math.atan2(rel_position.getY(), rel_position.getX());
        result.setLeftLegDir(new Vector2(Math.cos(ang_rel + max_ang), Math.sin(ang_rel + max_ang)));
        result.setRightLegDir(new Vector2(Math.cos(ang_rel + min_ang), Math.sin(ang_rel + min_ang)));

        result.setLeftLegDir(Vector2.rotateVectorByAngle(result.getLeftLegDir(), 0.15));
        result.setRightLegDir(Vector2.rotateVectorByAngle(result.getRightLegDir(), -0.05));


        result.setRelativePosition(rel_position);
        result.setCombinedRadius(Vector2.abs(rel_position) - min_dist);
        result.setPoint(new Vector2(0,0));

        result.setTruncLeft(intersectTwoLines(result.getPoint(), result.getLeftLegDir(), Vector2.mul(Vector2.normalize(rel_position),result.getCombinedRadius()/2.0 ), Vector2.minus(obst2,obst1)));
        result.setTruncRight(intersectTwoLines(result.getPoint(), result.getRightLegDir(), Vector2.mul(Vector2.normalize(rel_position),result.getCombinedRadius()/2.0 ), Vector2.minus(obst2,obst1)));

        return result;

    }

    static Vector2 intersectTwoLines(Vector2 point1, Vector2 dir1, Vector2 point2, Vector2 dir2) {
        double x1, x2, x3,x4, y1, y2,y3,y4;
        x1 = point1.getX();
        y1 = point1.getY();
        x2 = x1 + dir1.getX();
        y2 = y1 + dir1.getY();
        x3 = point2.getX();
        y3 = point2.getY();
        x4 = x3 + dir2.getX();
        y4 = y3 + dir2.getY();

        double det = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);

        if (det == 0) {
            return new Vector2(0,0); //TODO fix return NULL

        }
        double x_i = ( (x3-x4) * (x1*y2-y1*x2) - (x1-x2) * (x3*y4-y3*x4)) / det;
        double y_i = ( (y3-y4) * (x1*y2-y1*x2) - (y1-y2) * (x3*y4-y3*x4)) / det;

        return new Vector2(x_i, y_i);
    }


    //by footprint
    public static VO createVO(Vector2 position1, final List<Vector2> footprint1, Vector2 vel1, Vector2 position2, final List<Vector2> footprint2, Vector2 vel2, int TYPE) {
        if (TYPE == HRVOS) {
            return createHRVO(position1, footprint1, vel1, position2, footprint2, vel2);
        }
        else if (TYPE == RVOS) {
            return createRVO(position1, footprint1, vel1, position2, footprint2, vel2);
        }
        else{
            return createVO(position1, footprint1, position2, footprint2, vel2);
        }
    }

    //by radius
    public static VO createVO(Vector2 position1, double radius1, Vector2 vel1, Vector2 position2, double radius2, Vector2 vel2, int TYPE) {
        if(TYPE == HRVOS) {
            return createHRVO(position1, radius1, vel1, position2, radius2, vel2);
        }
        else if(TYPE == RVOS) {
            return createRVO(position1, radius1, vel1, position2, radius2, vel2);
        }
        else {
            return createVO(position1, radius1, position2, radius2, vel2);
        }
    }

    //different types
    //by footprint
    private static VO createHRVO(Vector2 position1, final List<Vector2> footprint1, Vector2 vel1, Vector2 position2, final List<Vector2> footprint2, Vector2 vel2){

        VO result = createRVO(position1, footprint1,vel1, position2, footprint2, vel2);
        Vector2 rel_velocity = Vector2.minus(vel1 , vel2);

        if (result.getCombinedRadius() > Vector2.abs(result.getRelativePosition())) {
            //ROS_ERROR("comb.rad. %f, relPos %f, %f (abs = %f)", result.combined_radius, result.relative_position.x(), result.relative_position.y(), abs(result.relative_position));
            return result;
        }

        if (leftOf(new Vector2(0.0, 0.0), result.getRelativePosition(), rel_velocity)) { //left of centerline
            result.setPoint(intersectTwoLines(result.getPoint(), result.getLeftLegDir(), vel2, result.getRightLegDir())); // TODO add uncertainty
        }
        else{ //right of centerline
            result.setPoint(intersectTwoLines(vel2,result.getLeftLegDir(), result.getPoint(), result.getRightLegDir())); // TODO add uncertainty
        }
        return result;
    }

    //by radius
    private static VO createHRVO(Vector2 position1, double radius1, Vector2 vel1, Vector2 position2, double radius2, Vector2 vel2){

        VO result = createRVO(position1, radius1, vel1, position2, radius2, vel2);

        Vector2 rel_velocity = Vector2.minus(vel1 , vel2);
        Vector2 rel_position = Vector2.minus(position2 , position1);
        if (Vector2.abs(rel_position) < radius1 + radius2){
            result.setPoint(Vector2.mul(Vector2.plus(vel2 , vel1),0.5));
            return result;
        }

        if (leftOf(new Vector2(0.0, 0.0), rel_position, rel_velocity)) { //left of centerline
            result.setPoint(intersectTwoLines(result.getPoint(), result.getLeftLegDir(), vel2, result.getRightLegDir())); // TODO add uncertainty
        }
        else{ //right of centerline
            result.setPoint(intersectTwoLines(vel2,result.getLeftLegDir(), result.getPoint(), result.getRightLegDir())); // TODO add uncertainty
        }
        return result;

    }

    //by footprint
    private static VO createRVO(Vector2 position1, final List<Vector2> footprint1, Vector2 vel1, Vector2 position2, final List<Vector2> footprint2, Vector2 vel2){
        VO result = createVO(position1, footprint1, position2, footprint2, vel2);
        result.setPoint(Vector2.mul(Vector2.plus(vel1 , vel2),0.5)); //TODO add uncertainty
        return result;
    }

    //by radius
    private static VO createRVO(Vector2 position1, double radius1, Vector2 vel1, Vector2 position2, double radius2, Vector2 vel2) {
        VO result = createVO(position1, radius1, position2, radius2, vel2);
        result.setPoint(Vector2.mul(Vector2.plus(vel1 , vel2),0.5)); //TODO add uncertainty
        return result;
    }

    //basic method by footprint
    private static VO createVO(Vector2 position1, List<Vector2> footprint1, Vector2 position2, List<Vector2> footprint2, Vector2 vel2){
        VO result=new VO();
        List<Vector2> mink_sum = minkowskiSum(footprint1, footprint2);

        Vector2 min_left=new Vector2();
        Vector2 min_right=new Vector2();
        double min_ang =  0.0;
        double max_ang = 0.0;
        Vector2 rel_position = Vector2.minus(position2 , position1);

        Vector2 rel_position_normal = Vector2.normal(rel_position);
        double min_dist = Vector2.abs(rel_position);

        for (int i = 0; i<  mink_sum.size(); i++){
            double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position, mink_sum.get(i)));
            if (rightOf(new Vector2(0.0,0.0), rel_position, Vector2.plus(rel_position , mink_sum.get(i)))) {
                if (-angle < min_ang) {
                    min_right = Vector2.plus(rel_position , mink_sum.get(i));
                    min_ang = -angle;
                }
            }
            else {
                if (angle > max_ang) {
                    min_left = Vector2.plus(rel_position , mink_sum.get(i));
                    max_ang = angle;
                }
            }
            Vector2 project_on_rel_position = intersectTwoLines(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position , mink_sum.get(i)), rel_position_normal);
            double dist = Vector2.abs(project_on_rel_position);
            if (Vector2.dotProduct(project_on_rel_position , rel_position) < -EPSILON.EPSILON){
                //	ROS_ERROR("Collision?");
                dist = -dist;

            }

            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist < 0) {
            result.setLeftLegDir(Vector2.negative(Vector2.normalize(rel_position_normal)));
            result.setRightLegDir(Vector2.negative(result.getLeftLegDir()));
            result.setRelativePosition(rel_position);
            result.setCombinedRadius(Vector2.abs(rel_position) - min_dist);
            result.setPoint(vel2);
            return result;
        }

        double ang_rel = Math.atan2(rel_position.getY(), rel_position.getX());
        result.setLeftLegDir(new Vector2(Math.cos(ang_rel + max_ang), Math.sin(ang_rel + max_ang)));
        result.setRightLegDir(new Vector2(Math.cos(ang_rel + min_ang), Math.sin(ang_rel + min_ang)));

        result.setLeftLegDir(Vector2.rotateVectorByAngle(result.getLeftLegDir(), 0.15));
        result.setRightLegDir(Vector2.rotateVectorByAngle(result.getRightLegDir(), -0.05));


        double ang_between = Vector2.angleBetween(result.getRightLegDir(), result.getLeftLegDir());
        double opening_ang = ang_rel + min_ang + (ang_between) / 2.0;

        Vector2 dir_center = new Vector2(Math.cos(opening_ang), Math.sin(opening_ang));
        min_dist = Vector2.abs(rel_position);
        Vector2 min_point = new Vector2(rel_position);
        for(int i = 0; i< mink_sum.size(); i++) {
            Vector2 proj_on_center = intersectTwoLines(new Vector2(0.0,0.0), dir_center, Vector2.plus(rel_position,mink_sum.get(i)), Vector2.normal(dir_center));
            if (Vector2.abs(proj_on_center) < min_dist) {
                min_dist = Vector2.abs(proj_on_center);
                min_point = Vector2.plus(rel_position,mink_sum.get(i));
            }

        }

        //ROS_ERROR("min_right %f, %f, min_left %f,%f,", min_right.x(), min_right.y(), min_left.x(), min_left.y());
        double center_p, radius;
        Vector2 center_r;
        //test left/right
        if ( Vector2.abs(min_left) < Vector2.abs(min_right)) {
            center_p = Vector2.abs(min_left) / Math.cos(ang_between / 2.0);
            radius = Math.tan(ang_between / 2.0) * Vector2.abs(min_left);
            center_r = Vector2.mul(dir_center,center_p);
        }
        else {
            center_p = Vector2.abs(min_right) / Math.cos(ang_between / 2.0);
            center_r = Vector2.mul(dir_center,center_p);
            radius = Math.tan(ang_between / 2.0) * Vector2.abs(min_right);
        }
        //check min_point, if failed stupid calc for new radius and point;
        Logger logger = Logger.getLogger("CreateVO");
        if (Vector2.abs(Vector2.minus(min_point , center_r)) > radius) {
            double gamma = min_point.getX() * dir_center.getX() + min_point.getY() * dir_center.getY();
            double sqrt_exp = Vector2.absSqr(min_point) / (Math.pow(Math.sin(ang_between / 2.0), 2) -1) + Math.pow(gamma / (Math.pow(Math.sin(ang_between / 2.0), 2) - 1), 2);
            if (Math.abs(sqrt_exp)<EPSILON.EPSILON) {
                sqrt_exp =0;
            }
            if (sqrt_exp >= 0) {
                center_p = - gamma / (Math.pow(Math.sin(ang_between/2.0),2) -1) + Math.sqrt(sqrt_exp);
                center_r = Vector2.mul(dir_center,center_p);
                radius = Vector2.abs(Vector2.minus(min_point, center_r));
            }
            else {
                logger.severe("CreateVO error: ang ="+ang_between+", sqrt_ext = "+"ang_between, sqrt_exp");
                logger.severe("CreateVO error: rel position "+rel_position.getX()+" "+rel_position.getY()+", radius "+radius+", center_p "+center_p);
            }
        }
        //result.relative_position = rel_position;
        //result.combined_radius = abs(rel_position) - min_dist;

        result.setRelativePosition( center_r);
        result.setCombinedRadius( radius);
        result.setPoint( vel2);
        return result;

    }

    //basic method by radius
    private static VO createVO(Vector2 position1, double radius1, Vector2 position2, double radius2, Vector2 vel2) {
        VO result=new VO();
        Vector2 rel_position = Vector2.minus(position2 , position1);
        double ang_to_other = Vector2.atan(rel_position);
        double combined_radius = radius2+radius1;
        double angle_of_opening;
        if (Vector2.abs(rel_position) < combined_radius) {

            result.setLeftLegDir(Vector2.negative(Vector2.normalize(Vector2.normal(rel_position))));
            result.setRightLegDir(Vector2.negative(result.getLeftLegDir()));
            }
        else {
            angle_of_opening = Math.asin(combined_radius / Vector2.abs(rel_position));
            result.setRightLegDir(new Vector2(Math.cos(ang_to_other - angle_of_opening), Math.sin(ang_to_other - angle_of_opening)));
            result.setLeftLegDir(new Vector2(Math.cos(ang_to_other + angle_of_opening), Math.sin(ang_to_other + angle_of_opening)));
        }
        //    ROS_ERROR("angle_of_opening %f, combined_radius %f, rel_position %f", angle_of_opening, combined_radius, abs(rel_position));
        result.setPoint(vel2);
        result.setRelativePosition(rel_position);
        result.setCombinedRadius( radius1 + radius2);

        return result;
    }


    public static VO createTruncVO (VO vo, double time) {
        VO result=new VO();
        result.setPoint( vo.getPoint());
        result.setLeftLegDir(vo.getLeftLegDir());
        result.setRightLegDir( vo.getRightLegDir());
        result.setRelativePosition( vo.getRelativePosition());
        result.setCombinedRadius( vo.getCombinedRadius());
        double trunc_radius = vo.getCombinedRadius() / time;
        double angle_of_opening;

        if (Vector2.abs(vo.getRelativePosition()) < vo.getCombinedRadius()) {
            result.setTruncLeft(result.getPoint());
            result.setTruncRight(result.getPoint());
            result.setTruncLineCenter(result.getPoint());
            return result;
        }
        else {
            angle_of_opening = Math.asin(vo.getCombinedRadius() / Vector2.abs(vo.getRelativePosition()));
            double trunc_dist = trunc_radius / Math.sin(angle_of_opening) - trunc_radius;
            result.setTruncLineCenter(Vector2.mul(Vector2.normalize(vo.getRelativePosition()), trunc_dist));
            Vector2 intersectLeft = intersectTwoLines(Vector2.plus(result.getPoint() , result.getTruncLineCenter()),
                    new Vector2(result.getTruncLineCenter().getY(), - result.getTruncLineCenter().getX()), result.getPoint(), result.getLeftLegDir());
            result.setTruncLeft(intersectLeft);
            result.setTruncRight(intersectTwoLines(Vector2.plus(result.getPoint(), result.getTruncLineCenter()),
                    new Vector2(result.getTruncLineCenter().getY(), -result.getTruncLineCenter().getX()), result.getPoint(), result.getRightLegDir()));

            return result;
        }
    }

}
