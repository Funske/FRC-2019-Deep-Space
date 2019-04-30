package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Intake extends Subsystem{

    private boolean grabDown = false;
    private boolean grabOpen = false;

    private DoubleSolenoid hatchIntake = new DoubleSolenoid(RobotMap.GRAB_IN, RobotMap.GRAB_OUT);
    private DoubleSolenoid hatchDrop = new DoubleSolenoid(RobotMap.EXTEND_IN, RobotMap.EXTEND_OUT);
    private TalonSRX intakeMotor = new TalonSRX(RobotMap.INTAKE);

    public double cargoSpeed = 1.0;

    @Override
    protected void initDefaultCommand() {
        
    }

    public Intake(){
        intakeMotor.configOpenloopRamp(0.2);
        intakeMotor.enableCurrentLimit(false);
        intakeMotor.configContinuousCurrentLimit(10);
        intakeMotor.configPeakCurrentLimit(15);
    }

    public void toggleHatchDrop(){
        System.out.println("Hatch Drop");
        grabDown = !grabDown;
        Robot.elevator.setCargoMode(!grabDown);
        if(grabDown)
            hatchDrop.set(Value.kForward);
        else{
            hatchDrop.set(Value.kReverse);
            setHatchIntake(true);
        }
    }

    public void setHatchDrop(boolean down){
        grabDown = down;
        Robot.elevator.setCargoMode(!grabDown);
        if(grabDown)
            hatchDrop.set(Value.kForward);
        else{
            hatchDrop.set(Value.kReverse);
            setHatchIntake(true);
        }
    }

    public void toggleHatchIntake(){
        grabOpen = !grabOpen;
        if(grabOpen)
            hatchIntake.set(Value.kForward);
        else
            hatchIntake.set(Value.kReverse);
    }

    public void setHatchIntake(boolean open){
        grabOpen = open;
        if(grabOpen)
            hatchIntake.set(Value.kForward);
        else
            hatchIntake.set(Value.kReverse);
    }

    public void setCargoMotor(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
}