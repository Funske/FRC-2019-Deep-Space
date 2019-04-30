package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ElevatorWidget;
import frc.robot.util.PIDData;
import frc.robot.util.RobotMath;

public class Elevator extends Subsystem{

    private ElevatorWidget widget = new ElevatorWidget("N/A", 0.0, 0);
    private int lastTargetLevel = 0;

    TalonSRX elevator = new TalonSRX(RobotMap.ELEVATOR);
    private final boolean sensorPhase = true;
    public double speed = 0.9;

    private boolean cargo_mode = true;
    private int level = 0;
    private double timeToClose = 0.25;
    private int lock_position = 0;

    public NetworkTableEntry going_upP = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Up P", 0).withPosition(1, 1).getEntry();
    public NetworkTableEntry going_upVel = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Up Velocity", 0).withPosition(1, 2).getEntry();
    public NetworkTableEntry going_upAcc = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Up Acceleration", 0).withPosition(1, 3).getEntry();
    public NetworkTableEntry going_upError = Shuffleboard.getTab("SmartDashboard").addPersistent("Up Error", 0).withPosition(1, 4).getEntry();

    public NetworkTableEntry going_downP = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Down P", 0).withPosition(2, 1).getEntry();
    public NetworkTableEntry going_downVel = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Down Velocity", 0).withPosition(2, 2).getEntry();
    public NetworkTableEntry going_downAcc = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Down Acceleration", 0).withPosition(2, 3).getEntry();
    public NetworkTableEntry going_downError = Shuffleboard.getTab("SmartDashboard").addPersistent("Down Error", 0).withPosition(2, 4).getEntry();

    public NetworkTableEntry going_level3P = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Level 3 P", 0).withPosition(4, 1).getEntry();
    public NetworkTableEntry going_level3Vel = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Level 3 Velocity", 0).withPosition(4, 2).getEntry();
    public NetworkTableEntry going_level3Acc = Shuffleboard.getTab("SmartDashboard").addPersistent("Going Level 3 Acceleration", 0).withPosition(4, 3).getEntry();
    public NetworkTableEntry going_level3Error = Shuffleboard.getTab("SmartDashboard").addPersistent("Level 3 Error", 0).withPosition(4, 4).getEntry();

    public NetworkTableEntry level0 = Shuffleboard.getTab("SmartDashboard").addPersistent("Level 0", 0).withPosition(3, 1).getEntry();
    public NetworkTableEntry level1 = Shuffleboard.getTab("SmartDashboard").addPersistent("Level 1", 0).withPosition(3, 2).getEntry();
    public NetworkTableEntry level2 = Shuffleboard.getTab("SmartDashboard").addPersistent("Level 2", 0).withPosition(3, 3).getEntry();

    public PIDData going_up = new PIDData(0.2, 0, 0.02, 0, 0, 10);
    public PIDData going_down = new PIDData(0.2, 0, 0, 0, 1, 10);
    public PIDData going_level3 = new PIDData(0.2, 0, 0, 0, 2, 10);

    @Override
    protected void initDefaultCommand() {
        
    }

    public Elevator(){
        elevator.setNeutralMode(NeutralMode.Brake);
        elevator.configOpenloopRamp(0.5);
        elevator.configNeutralDeadband(0.09);
        
        elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.TIMEOUT);
        elevator.setSensorPhase(sensorPhase);
        elevator.configClosedloopRamp(timeToClose);

        going_up.setMaxVelocity(6000);
        going_up.setMaxAcceleration(5000);

        going_down.setMaxVelocity(2000);
        going_down.setMaxAcceleration(2000);

        going_level3.setMaxVelocity(6000);
        going_level3.setMaxAcceleration(6000);

        RobotMath.setPID(elevator, going_up);
        RobotMath.setPID(elevator, going_down);
        RobotMath.setPID(elevator, going_level3);
    }

    public void elevatorPeriodic(){
        if(widget.getTargetLevel() != lastTargetLevel)//There has been a change in the level target
            setLevel(widget.getTargetLevel());

        if(widget.getEncoderPosition() == 0 && getPosition() != 0)
            setPosition(0);

        lastTargetLevel = widget.getTargetLevel();
    }

    public void setPD(){
        /*going_up = new PIDData(going_upP.getDouble(0.1), 0, 0, 0, 0, 10);
        going_up.setMaxVelocity((int)going_upVel.getDouble(400));
        going_up.setMaxAcceleration((int)going_upAcc.getDouble(400));
        going_up.setMaxError((int)going_upError.getDouble(0));

        going_down = new PIDData(going_downP.getDouble(0.1), 0, 0, 0, 1, 10);
        going_down.setMaxVelocity((int)going_downVel.getDouble(400));
        going_down.setMaxAcceleration((int)going_downAcc.getDouble(400));
        going_down.setMaxError((int)going_downError.getDouble(0));

        going_level3 = new PIDData(going_level3P.getDouble(0.1), 0, 0, 0, 2, 10);
        going_level3.setMaxVelocity((int)going_level3Vel.getDouble(400));
        going_level3.setMaxAcceleration((int)going_level3Acc.getDouble(400));
        going_level3.setMaxAcceleration((int)going_level3Error.getDouble(0));*/
        
        //RobotMath.setPID(elevator, going_up);
        //RobotMath.setPID(elevator, going_down);
        //RobotMath.setPID(elevator, going_level3);
    }

    public void setPercent(double percent){
        elevator.set(ControlMode.PercentOutput, percent);
    }

    public void lockPos(){
        lock_position = getPosition();
        setMaxVelocity(going_up.getMaxVelocity());
        setMaxAcceleration(going_up.getMaxAcceleration());
        elevator.selectProfileSlot(going_up.getSlotIndex(), 0);

        setPosition(lock_position);
    }

    public void setPosition(double position){
        elevator.set(ControlMode.MotionMagic, position);
    }

    public void resetPosition(){
        elevator.setSelectedSensorPosition(0);
    }

    public void setCoast(){
        elevator.setNeutralMode(NeutralMode.Coast);
    }

    public void setMaxVelocity(int velocity){
        elevator.configMotionCruiseVelocity(velocity);
    }

    public void setMaxAcceleration(int acceleration){
        elevator.configMotionAcceleration(acceleration);
    }

    public int getPosition(){
        return elevator.getSelectedSensorPosition();
    }

    public int getVelocity(){
        return elevator.getSelectedSensorVelocity();
    }

    public int getLevel(){
        return level;
    }

    public void setCargoMode(boolean mode){
        cargo_mode = mode;
    }

    public boolean getCargoMode(){
        return cargo_mode;
    }

    public void increaseLevel(){
        level++;
        if(level > 3)
            level = 3;
        setLevel(level);
    }

    public void decreaseLevel(){
        level--;
        if(level < 0)
            level = 0;
        setLevel(level);
    }

    public void setLevel(int level){
        if(level == 5 && this.level >= 2){//We are trying to go to cargoship when we are too high
            return;
        }

        System.out.println("Level: " + level);
        elevator.setNeutralMode(NeutralMode.Brake);

        if(level < this.level || this.level == 0){//Going Down
            System.out.println("Going Down");
            setMaxVelocity(going_down.getMaxVelocity());
            setMaxAcceleration(going_down.getMaxAcceleration());
            elevator.selectProfileSlot(going_down.getSlotIndex(), 0);
        }else if(level > this.level && level != 3){//Going Up
            System.out.println("Going Up");
            setMaxVelocity(going_up.getMaxVelocity());
            setMaxAcceleration(going_up.getMaxAcceleration());
            elevator.selectProfileSlot(going_up.getSlotIndex(), 0);
        }else if(level == 3){//Going to level 3
            System.out.println("Going To Level 3");
            setMaxVelocity(going_level3.getMaxVelocity());
            setMaxAcceleration(going_level3.getMaxAcceleration());
            elevator.selectProfileSlot(going_level3.getSlotIndex(), 0);
        }

        this.level = level;
        switch(level){
            case 0:
                if(cargo_mode){
                    setPosition(RobotMap.LEVEL0_POS);//Rest on the ground
                }else{
                    setPosition(RobotMap.LEVEL0_POS_HATCH);//Rest at the loading station
                }
                //setPosition(level0.getDouble(0));
                Robot.driveTrain.setMaxSpeed(1.0);
            break;
            case 1:
                if(cargo_mode){
                    setPosition(RobotMap.LEVEL1_POS);
                }else{
                    setPosition(RobotMap.LEVEL1_POS_HATCH);
                }
                //setPosition(level1.getDouble(5*RobotMap.ENCODER_UNITS));
                Robot.driveTrain.setMaxSpeed(1.0);
            break;
            case 2:
                if(cargo_mode){
                    setPosition(RobotMap.LEVEL2_POS);
                }else{
                    setPosition(RobotMap.LEVEL2_POS_HATCH);
                }
                //setPosition(level2.getDouble(10*RobotMap.ENCODER_UNITS));
                Robot.driveTrain.setMaxSpeed(0.75);
            break;
            case 3:
                if(cargo_mode){
                    setPosition(RobotMap.LEVEL3_POS);
                }else{
                    setPosition(RobotMap.LEVEL3_POS_HATCH);
                }
                Robot.driveTrain.setMaxSpeed(0.25);
            break;
            case 5://"5" looks like an "S" for "Ship"
                if(cargo_mode){
                    setPosition(RobotMap.LEVEL_CARGO);
                }
                Robot.driveTrain.setMaxSpeed(1.0);
            break;
        }
    }

    public double getCurrent(){
        return elevator.getOutputCurrent();
    }

    public void updateWidget(){
        widget.setEncoderPosition(getPosition());
        widget.setLevelText(getWidgetLevel());
    }

    public String getWidgetLevel(){
        int pos = getPosition();
        if(pos < 1024*2){
            return "Level: Ground";
        }else if(pos > 5000 && pos < 11000 && !cargo_mode){
            return "Level: Loading Station";
        }else if(pos > 35000 && pos < 42500 && !cargo_mode){
            return "Level: Rocket 2";
        }else if(pos > 65000 && pos > 72000 && !cargo_mode){
            return "Level: Rocket 3";
        }else if(pos > 26000 && pos < 30000 && cargo_mode){
            return "Level: Rocket 1";
        }else if(pos > 60000 && pos < 64000 && cargo_mode){
            return "Level: Rocket 2";
        }else if(pos > 87000 && pos < 91000 && cargo_mode){
            return "Level: Rocket 3";
        }else if(pos > 47000 && pos < 50000 && cargo_mode){
            return "Level: Cargo Ship";
        }

        return "N/A";
    }

    public ElevatorWidget getWidget(){
        return widget;
    }
}