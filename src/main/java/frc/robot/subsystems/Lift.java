package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Lift extends Subsystem{

    public boolean isExtended = false;
    public DoubleSolenoid lift = new DoubleSolenoid(RobotMap.LIFT_IN, RobotMap.LIFT_OUT);

    @Override
    protected void initDefaultCommand() {
        
    }

    public void toggleLift(){
        isExtended = !isExtended;
        if(isExtended)
            lift.set(Value.kForward);
        else
            lift.set(Value.kReverse);
    }

    public void setExtend(boolean extend){
        isExtended = extend;
        if(isExtended)
            lift.set(Value.kForward);
        else
            lift.set(Value.kReverse);
    }

}