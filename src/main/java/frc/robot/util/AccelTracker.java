package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;

public class AccelTracker implements Runnable{

    public static final double GRAVITY = 9.80665;//Gravity in m/s^2
    public static final double FIXED_POINT = 16384;
    public static final double FIXED_TO_METERS = FIXED_POINT/GRAVITY;

    public static final double xCalibrate = 270;
    public static final double yCalibrate = -27;
    public static final double zCalibrate = 17000;

    private double lastXAccel = 0, lastYAccel = 0;
    private double lastXVelocity = 0, lastYVelocity = 0;

    public double xPos = 0, yPos = 0;
    private double lastXPos = 0, lastYPos = 0;

    NetworkTableEntry networkX = Shuffleboard.getTab("SmartDashboard").add("X Position", 0).getEntry();
    NetworkTableEntry networkY = Shuffleboard.getTab("SmartDashboard").add("Y Position", 0).getEntry();

    private double mult = 0.01;

    public void run(){
        double xAccel = getXLinear()/FIXED_TO_METERS;
        double yAccel = getYLinear()/FIXED_TO_METERS;

        if(xAccel < 0.1 && xAccel > -0.1){
            xAccel = 0;
            lastXVelocity = 0;
            lastXPos = 0;
        }if(yAccel < 0.1 && yAccel > -0.1){
            yAccel = 0;
            lastYVelocity = 0;
            lastYPos = 0;
        }

        double xVel = lastXAccel + lastXVelocity + ((xAccel - lastXAccel)/2);
        double yVel = lastYAccel + lastYVelocity + ((yAccel - lastYAccel)/2);

        xPos += lastXVelocity + lastXPos + ((xVel - lastXVelocity)/2);
        yPos += lastYVelocity + lastYPos + ((yVel - lastYVelocity)/2);

        lastXAccel = xAccel;
        lastYAccel = yAccel;

        lastXVelocity = xVel;
        lastYVelocity = yVel;

        lastXPos = xPos;
        lastYPos = yPos;

        //xPos += xTemp;
        //yPos += yTemp;

        networkX.forceSetDouble(xVel+mult);
        networkY.forceSetDouble(xPos+mult);
        mult *= -1;
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