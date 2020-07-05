/* 
 * File:   pose.h
 * Author: greg
 *
 * Created on 4. december 2012, 15:03
 */

#ifndef POSE_H
#define	POSE_H

#include <math.h>

struct Vele //Vele is to Velocity as Pose is to Position
{
    double u;
    double v;
    double omega;
    
    Vele(void)
    {
        u = v = omega = 0;
    }
    
    Vele(double U, double V, double om)
    : u(U), v(V), omega(om)//*M_PI / 180.)
    {
    }
};

struct Pose
{
    double x;
    double y;
    double theta;

    Pose(void)
    {
        x = y = theta = 0;
    }

    Pose(double X, double Y, double th)
        : x(X), y(Y), theta(th)
    {}

    double CalcDistanceToPose(const Pose& p) const
    {
        double sqd = (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y);
        return sqrt(sqd);
    }

    double CalcAngleToPose(const Pose& p) const
    //angle to p
    {
        double ang = atan2(p.y - y, p.x - x);
        return ang;
    }

    double CalcAngleBetweenPose(const Pose& p)
    //difference in orientation
    {
        float ang = (p.theta - theta);
        return ang;
    }

    Pose CalcDifferenceAsPose(const Pose& p) const
    {
        return Pose(p.x - x, p.y - y, p.theta - theta);
    }

    Pose UpdatePose(const Vele& vel, double dt)//v is in LOCAL coordinates
    {
        //update position first, then the angle (could be smarter)
        x += (vel.u * cos(theta) - vel.v * sin(theta)) * dt;
        y += (vel.v * cos(theta) + vel.u * sin(theta)) * dt;

        theta += vel.omega * dt;

        return *this;
    }

    Pose CalcPoseByDistanceInFrontOf(float dist, float angle = 0)
    {
        Pose view;
        view.x = x + dist * cos(theta + angle);
        view.y = y + dist * sin(theta + angle);
        view.theta = theta + angle;

        return view;
    }

    /*
     * Converts a Pose in local coordinates (relative to this one) to global
     */
    Pose CalcGlobalPoseFromLocal(const Pose& local) const
    {
    	Pose global;
        global.x = x + (local.x * cos(theta) - local.y * sin(theta));
        global.y = y + (local.y * cos(theta) + local.x * sin(theta));
        global.theta = theta + local.theta;

        return global;
    }

};

#endif	/* POSE_H */

