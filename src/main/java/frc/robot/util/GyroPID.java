package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroPID{

    private double range = 1;//Angle range
    private double pGain = 0.1, dGain = 0;
    private double setpoint = 0, zero = 0;

    private double max_input = 180;

    private boolean has_setpoint = false, finished = false;

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
        has_setpoint = true;
    }

    public void setZero(double zero){
        this.zero = zero;
    }

    public double pidUpdate(double input_angle, double angle_rate){
        double error = setpoint - (input_angle-zero);
        double output = ((error * pGain) + (angle_rate * dGain))*range;

        if(output > range)
            output = range;
        else if(output < -range)
            output = -range;
        return output;
    }

    public boolean hasSetpoint(){
        return has_setpoint;
    }

    public boolean isFinished(){
        return finished;
    }
}