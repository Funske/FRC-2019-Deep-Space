package frc.robot.util;

/*
****
Code Written By Team 3691

Inspired by team 254 2017 robot program: https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/lib/util/math/RigidTransform2d.java
****
*/
public class Position{

    private final double kEpsiolon = 1E-9;
    private double x, y;
    private double xComp, yComp;

    public Position(double x, double y, double xComp, double yComp){
        this.x = x;
        this.y = y;
        this.xComp = xComp;
        this.yComp = yComp;
    }

    public void rotateBy(double dx, double dy){
        /*xComp = xComp*deltaX - yComp*deltaY;
        yComp = xComp*deltaX + yComp*deltaY;
        normalize();*/
        x = x*dx - y*dy;
        y = x*dy + y*dx;
    }

    public void translate(double dx, double dy){
        x += dx;
        y += dy;
    }

    /*
    public static RigidTransform2d.Delta forwardKinematics(double left_wheel_delta, double right_wheel_delta,
            double delta_rotation_rads) {
        return new RigidTransform2d.Delta((left_wheel_delta + right_wheel_delta) / 2, 0, delta_rotation_rads);
    }

    /** Append the result of forward kinematics to a previous pose.
        public static RigidTransform2d integrateForwardKinematics(RigidTransform2d current_pose, double left_wheel_delta,
                double right_wheel_delta, Rotation2d current_heading) {
            RigidTransform2d.Delta with_gyro = forwardKinematics(left_wheel_delta, right_wheel_delta,
                    current_pose.getRotation().inverse().rotateBy(current_heading).getRadians());
            return current_pose.transformBy(RigidTransform2d.fromVelocity(with_gyro));
    }
    */

    public Position fromVelocity(double dx, double dy, double dtheta){
        double dsin = Math.sin(dtheta);
        double dcos = Math.cos(dtheta);
        double sin, cos;
        if(Math.abs(dtheta) < kEpsiolon){
            sin = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            cos = 0.5 * dtheta;
        } else {
            sin = dsin / dtheta;
            cos = (1.0 - dcos) / dtheta;
        }

        return new Position(dx*sin - dy*cos, dx*cos + dy*sin, dcos, dsin);
    }

    public void normalize(){
        double mag = Math.hypot(xComp, yComp);
        if(mag > kEpsiolon){
            xComp /= mag;
            yComp /= mag;
        }else{
            yComp = 0;
            xComp = 1;
        }
    }
}