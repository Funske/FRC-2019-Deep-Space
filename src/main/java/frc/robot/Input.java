package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Input{

    public Joystick controller = new Joystick(0);
    public Joystick sysController = new Joystick(1);
    int xAxis = 0, yAxis = 3;

    public double getXAxis(){
        if(controller.getRawAxis(xAxis) < RobotMap.DEADBAND)
             return 0;
        return controller.getRawAxis(xAxis);
    }

    public double getYAxis(){
        if(controller.getRawAxis(yAxis) < RobotMap.DEADBAND)
             return 0;
        return controller.getRawAxis(yAxis);
    }

    public double getLineSpeed(){
        return (sysController.getZ()+1)/2;//Use the Z axis but normalize it from [-1, 1] to [0, 1]
    }
}