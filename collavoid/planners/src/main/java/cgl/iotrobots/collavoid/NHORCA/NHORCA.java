package cgl.iotrobots.collavoid.NHORCA;

import cgl.iotrobots.collavoid.utils.EPSILON;
import cgl.iotrobots.collavoid.utils.Line;
import cgl.iotrobots.collavoid.utils.Vector2;

import java.util.List;

import static cgl.iotrobots.collavoid.utils.utils.sign;


//some of the implementations of formulation 13 in the NHORCA paper, add orcalines coursed by NH constraints

public class NHORCA {
    //from orca
    //in velocity space
    public static void addMovementConstraintsDiffSimple(double max_track_speed, double heading, List<Line> additional_orca_lines) {
        Line maxVel1;
        Line maxVel2;

        //if angle difference is more than PI/2 then set angle difference to PI/2 and
        //this is the max holo velocity that can track.
        Vector2 dir = new Vector2(Math.cos(heading), Math.sin(heading));
        Vector2 pt=Vector2.mul(new Vector2(-dir.getY(),dir.getX()),0.95*max_track_speed);
        maxVel1=new Line(pt,Vector2.negative(dir));
        pt=Vector2.mul(new Vector2(-dir.getY(),dir.getX()),-max_track_speed);
        maxVel2=new Line(pt,dir);

        additional_orca_lines.add(maxVel1);
        additional_orca_lines.add(maxVel2);

    }

    //from orca
    public static void addMovementConstraintsDiff(double error, double T,  double max_vel_x, double max_vel_th, double heading, double v_max_ang, List<Line> additional_orca_lines){
        // the max tracking angle
        double min_theta = Math.PI / 2.0;
        double max_track_speed = calculateMaxTrackSpeedAngle(T,min_theta, error, max_vel_x, max_vel_th, v_max_ang);

        //normal vector of heading, clock wise
        Vector2 first_point = new Vector2(Math.cos(heading - min_theta),Math.sin(heading - min_theta));
        first_point.mul(max_track_speed);

        double steps = 10.0;
        double step_size = - Math.PI / steps;
        // set all holo velocity that can be tracked for angle from heading-PI/2 to heading+PI/2
        for (int i=1; i<=(int)steps; i++) {
            Line line;
            double cur_ang = min_theta + i* step_size;
            Vector2 second_point = new Vector2(Math.cos(heading - cur_ang), Math.sin(heading - cur_ang));
            double track_speed = calculateMaxTrackSpeedAngle(T,cur_ang, error, max_vel_x, max_vel_th, v_max_ang);
            second_point.setVector2(Vector2.mul(second_point,track_speed));

            line=new Line(first_point,Vector2.normalize(Vector2.minus(second_point, first_point)));
            additional_orca_lines.add(line);
            //    ROS_DEBUG("line point 1 x, y, %f, %f, point 2 = %f,%f",first_point.x(),first_point.y(),second_point.x(),second_point.y());
            first_point.setVector2(second_point);
        }
    }

    public static void addAccelerationConstraintsXY(double max_vel_x, double acc_lim_x, double max_vel_y, double acc_lim_y, Vector2 cur_vel, double heading, double sim_period, boolean holo_robot, List<Line> additional_orca_lines){
        double max_lim_x, max_lim_y, min_lim_x, min_lim_y;
        Line line_x_back, line_x_front, line_y_left, line_y_right;

        Vector2 dir_front =  new Vector2(Math.cos(heading),Math.sin(heading));
        Vector2 dir_right =  new Vector2(dir_front.getY(),-dir_front.getX());
        if(holo_robot) {

            cur_vel = Vector2.rotateVectorByAngle(cur_vel, -heading);

            double cur_x = cur_vel.getX();
            double cur_y = cur_vel.getY();

            max_lim_x = Math.min(max_vel_x, cur_x + sim_period * acc_lim_x);
            max_lim_y = Math.min(max_vel_y, cur_y + sim_period * acc_lim_y);
            min_lim_x = Math.max(-max_vel_x, cur_x - sim_period * acc_lim_x);
            min_lim_y = Math.max(-max_vel_y, cur_y - sim_period * acc_lim_y);

            //      ROS_ERROR("Cur cvel (%f, %f) Max limints x = %f, %f, y = %f, %f", cur_x, cur_y, max_lim_x, min_lim_x, max_lim_y, min_lim_y);

            line_x_front=new Line(Vector2.mul(dir_front,max_lim_x),Vector2.negative(dir_right));

            line_x_back=new Line(Vector2.mul(dir_front,min_lim_x),dir_right);

            additional_orca_lines.add(line_x_front);
            additional_orca_lines.add(line_x_back);

            line_y_left=new Line(Vector2.mul(Vector2.negative(dir_right), max_lim_y),Vector2.negative(dir_front));

            line_y_right=new Line(Vector2.mul(Vector2.negative(dir_right),min_lim_y),dir_front);


            additional_orca_lines.add(line_y_left);
            additional_orca_lines.add(line_y_right);
        }
        else {

            double cur_x = Vector2.abs(cur_vel);
            max_lim_x = Math.min(max_vel_x, cur_x + sim_period * acc_lim_x);
            min_lim_x = Math.max(-max_vel_x, cur_x - sim_period * acc_lim_x);

            line_x_front=new Line(Vector2.mul(dir_front,max_lim_x),Vector2.negative(dir_right));

            line_x_back=new Line(Vector2.mul(dir_front,min_lim_x),dir_right);

            //ROS_ERROR("Max limints x = %f, %f", max_lim_x, min_lim_x);
            additional_orca_lines.add(line_x_front);
            additional_orca_lines.add(line_x_back);


        }
    }


    //from original program orca, parameters are corresponding to the NHrobot reference
    //                                                      T        theta_H        e               v_max             w                 v_max_w
    public static double calculateMaxTrackSpeedAngle(double T, double theta, double error, double max_vel_x, double max_vel_th, double v_max_ang){
        if (Math.abs(theta) <= EPSILON.EPSILON)
            return max_vel_x;
        if (Math.abs(theta / T) <= max_vel_th) {// formulation 13 in paper NHORCA
            double vstar_error = NHORCA.calcVstarError(T, theta, error);
            double vstar=calcVstar(error/T,theta);
            if (vstar_error <= v_max_ang) {
                //return 0;
                return Math.min(vstar_error /vstar, max_vel_x);
            }
            else {
                double a, b, g;
                a = T * T;
                b = beta(T, theta, v_max_ang);
                g = gamma(T, theta, error, v_max_ang);
                //return 1;
                //different from the reference not sure right or wrong so stick to the original reference
                //return Math.min((-b + Math.sqrt(Math.pow(b, 2) - 4 * Math.pow(a, 2) * g)) / (2.0 * g), max_vel_x);
                return Math.min((-b + Math.sqrt(Math.pow(b, 2) - 4 * a * g)) / (2.0 * g), max_vel_x);
            }
        }
        else
            //  return 2;
        //set theta to positive
            return Math.min(sign(theta) * error * max_vel_th / theta, max_vel_x);
    }

    public static double beta(double T, double theta, double v_max_ang){
        return - ((2.0 * T * T * Math.sin(theta)) / theta) * v_max_ang;
    }

    public static double gamma(double T, double theta, double error, double v_max_ang) {
        return ((2.0 * T * T * (1.0 - Math.cos(theta))) / (theta * theta)) * v_max_ang * v_max_ang - error * error;
    }

    public static double calcVstar(double vh, double theta){
        return vh * ((theta * Math.sin(theta))/(2.0 * (1.0 - Math.cos(theta))));
    }

    public static double calcVstarError(double T,double theta, double error) {
        return calcVstar(error/T,theta) * Math.sqrt((2.0 * (1.0 - Math.cos(theta))) / (2.0 * (1.0 - Math.cos(theta)) - Math.pow(Math.sin(theta), 2)));
    }

}
