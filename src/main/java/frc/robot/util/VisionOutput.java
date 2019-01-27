package frc.robot.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class VisionOutput implements PIDOutput{

    private double output;

    public void pidWrite(double output){
        this.output = output;
    }

    public double getOutput(){
        return output;
    }
}