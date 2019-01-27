package frc.robot.util;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Robot;

public class GyroSource implements PIDSource{

    private PIDSourceType pidSource = PIDSourceType.kDisplacement;
    private double reset = 0;

    public double pidGet(){
        return reset-Robot.driveTrain.getAngle();
    } 

    public double getReset(){
        return reset;
    }

    public void setReset(double reset){
        this.reset = reset;
    }

    public void setPIDSourceType(PIDSourceType pidSource){
        this.pidSource = pidSource;
    }

    public PIDSourceType getPIDSourceType(){
        return pidSource;
    }

}