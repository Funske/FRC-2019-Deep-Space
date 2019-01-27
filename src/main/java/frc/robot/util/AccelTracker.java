package frc.robot.util;

import frc.robot.Robot;

public class AccelTracker implements Runnable{

    public static final double xCalibrate = 0;
    public static final double yCalibrate = 0;
    public static final double zCalibrate = 0;

    public void run(){
        
    }

    public static void Calibrate(){
        int count = 1024;
        short[] data = new short[3];

        double xVal = 0, yVal = 0, zVal = 0;

        for(int i = 0; i < count; i++){
            Robot.driveTrain.pigeon.getBiasedAccelerometer(data);
            xVal += data[0]; 
            yVal += data[1];
            zVal += data[2];
        }

        System.out.println("X Result: " + (xVal/count));
        System.out.println("Y Result: " + (yVal/count));
        System.out.println("Z Result: " + (zVal/count));
    }

    public static double getXLinear(){
        short[] xyzAccel = new short[3];
        Robot.driveTrain.pigeon.getBiasedAccelerometer(xyzAccel);
        return xyzAccel[0]-xCalibrate;
    }

    public static double getYLinear(){
        short[] xyzAccel = new short[3];
        Robot.driveTrain.pigeon.getBiasedAccelerometer(xyzAccel);
        return xyzAccel[1]-yCalibrate;
    }

    public static double getZLinear(){
        short[] xyzAccel = new short[3];
        Robot.driveTrain.pigeon.getBiasedAccelerometer(xyzAccel);
        return xyzAccel[2]-zCalibrate;
    }
}