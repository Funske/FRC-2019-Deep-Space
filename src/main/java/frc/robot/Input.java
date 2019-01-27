package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Input{

    public Joystick controller = new Joystick(0);
    public Joystick sysController = new Joystick(1);
    int xAxis = 1, yAxis = 3;

    public double getXAxis(){
        //if(Math.abs(controller.getRawAxis(xAxis)) < RobotMap.DEADBAND)
             //return 0;
        //return controller.getRawAxis(xAxis);
        return -controller.getY();
    }

    public double getYAxis(){
        //if(Math.abs(controller.getRawAxis(yAxis)) < RobotMap.DEADBAND)
            // return 0;
        //return controller.getRawAxis(yAxis);
        return -sysController.getY();
    }

    public double getLineSpeed(){
        //return (sysController.getZ()+1)/2;//Use the Z axis but normalize it from [-1, 1] to [0, 1]
        return 0.2;
    }

    public boolean getVisionButton(){
        return controller.getRawButton(2);
    }

    public boolean cancelVision(){
        return controller.getRawButtonPressed(3);
    }
}