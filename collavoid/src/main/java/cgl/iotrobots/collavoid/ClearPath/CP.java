package cgl.iotrobots.collavoid.ClearPath;

import cgl.iotrobots.collavoid.utils.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

import static cgl.iotrobots.collavoid.utils.utils.leftOf;
import static cgl.iotrobots.collavoid.utils.utils.rightOf;

/**
 * Created by hjh on 11/1/14.
 */
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
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].point, truncated_vos[i].left_leg_dir, additional_constraints[j].point, additional_constraints[j].dir, max_speed, RAYLINE);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].point, truncated_vos[i].right_leg_dir, additional_constraints[j].point, additional_constraints[j].dir, max_speed, RAYLINE);
                }
                else {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_left, truncated_vos[i].left_leg_dir, additional_constraints[j].point, additional_constraints[j].dir, max_speed, RAYLINE);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_right, truncated_vos[i].right_leg_dir, additional_constraints[j].point, additional_constraints[j].dir, max_speed, RAYLINE);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_left, truncated_vos[i].trunc_right - truncated_vos[i].trunc_left, additional_constraints[j].point, additional_constraints[j].dir, max_speed, SEGMENTLINE);
                }
            }

            for (int j = i+1; j <  truncated_vos.size(); j++){

                if (!use_truncation) {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].point, truncated_vos[i].left_leg_dir, truncated_vos[j].point, truncated_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].point, truncated_vos[i].left_leg_dir, truncated_vos[j].point, truncated_vos[j].right_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].point, truncated_vos[i].right_leg_dir, truncated_vos[j].point, truncated_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].point, truncated_vos[i].right_leg_dir, truncated_vos[j].point, truncated_vos[j].right_leg_dir, max_speed, RAYRAY);

                }
                else {
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_left, truncated_vos[i].left_leg_dir, truncated_vos[j].trunc_left, truncated_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_left, truncated_vos[i].left_leg_dir, truncated_vos[j].trunc_right, truncated_vos[j].right_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_right, truncated_vos[i].right_leg_dir, truncated_vos[j].trunc_left, truncated_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_right, truncated_vos[i].right_leg_dir, truncated_vos[j].trunc_left, truncated_vos[j].right_leg_dir, max_speed, RAYRAY);


                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_left, truncated_vos[i].left_leg_dir, truncated_vos[j].trunc_left, truncated_vos[j].trunc_right - truncated_vos[j].trunc_left, max_speed, RAYSEGMENT); //left trunc
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[j].trunc_left, truncated_vos[j].left_leg_dir, truncated_vos[i].trunc_left, truncated_vos[i].trunc_right - truncated_vos[i].trunc_left, max_speed, RAYSEGMENT); //trunc left

                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_right, truncated_vos[i].right_leg_dir, truncated_vos[j].trunc_left, truncated_vos[j].trunc_right - truncated_vos[j].trunc_left, max_speed, RAYSEGMENT); //right trunc
                    addRayVelocitySamples(samples, pref_vel, truncated_vos[j].trunc_right, truncated_vos[j].right_leg_dir, truncated_vos[i].trunc_left, truncated_vos[i].trunc_right - truncated_vos[i].trunc_left, max_speed, RAYSEGMENT); //trunc right

                    addRayVelocitySamples(samples, pref_vel, truncated_vos[i].trunc_left, truncated_vos[i].trunc_right - truncated_vos[i].trunc_left, truncated_vos[j].trunc_left, truncated_vos[j].trunc_right - truncated_vos[j].trunc_left, max_speed, SEGMENTSEGMENT); //trunc trunc


                }

            }
        }


        //    ROS_ERROR("projection list length  = %d", samples.size());

        std::sort(samples.begin(), samples.end(), compareVelocitySamples);

        Vector2 new_vel; // = pref_vel;

        boolean valid = false;
        boolean foundOutside = false;
        boolean outside = true;
        int optimal = -1;

        for (int i = 0; i < (int) samples.size(); i++) {
            outside = true;
            valid = true;
            if (!isWithinAdditionalConstraints(additional_constraints, samples[i].velocity) ) {
                outside = false;
            }


            for (int j = 0; j < (int) truncated_vos.size(); j++) {
                if (isInsideVO(truncated_vos[j], samples[i].velocity, use_truncation)) {
                    valid = false;
                    if (j> optimal) {
                        optimal = j;
                        new_vel = samples[i].velocity;
                    }
                    break;
                }
            }
            if (valid && outside){
                return samples[i].velocity;
            }
            if (valid && !outside && !foundOutside) {
                optimal = truncated_vos.size();
                new_vel = samples[i].velocity;
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
        result.setLeftLegDir(Vector2.rotateVectorByAngle(result.getRightLegDir(), -0.05));


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

}
