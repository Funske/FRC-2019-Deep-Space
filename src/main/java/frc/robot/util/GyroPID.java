package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class GyroPID{

    private double range = 1;
    private double pGain = 0.1, dGain = 0;
    private double setpoint = 0, zero = 0;

    private double max_input = 180;

    public GyroPID(double range){
        this.range = range;
    }

    public void setPID(double pGain, double dGain){
        this.pGain = pGain;
        this.dGain = dGain;
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
        SmartDashboard.putNumber("Gyro Setpoint", setpoint);
    }

    public void setZero(double zero){
        this.zero = zero;
    }

    public double pidUpdate(){
        double error = setpoint - (Robot.driveTrain.getAngle()-zero);
        double output = ((error * pGain) + (Robot.driveTrain.getGyroRate() * dGain))*range;

        if(output > range)
            output = range;
        else if(output < -range)
            output = -range;
        return output;
    }
}