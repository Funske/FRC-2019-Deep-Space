package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class DriveTrain extends Subsystem{

    TalonSRX left_front, right_front;
    VictorSPX left_rear, right_rear;
    PigeonIMU pigeon;
    DigitalInput lineC = new DigitalInput(0), lineL = new DigitalInput(1), lineR = new DigitalInput(2);

    public boolean slaveSet = false, pidSet = false;
    public boolean highGear = true;
    private boolean leftPhase = false, rightPhase = false;

    public double teleopSpeed = 1.0, autoSpeed = 0.5;
    public static enum DriverStatus{Control, Distance, Locate, Follow};
    public DriverStatus status = DriverStatus.Control;

    public void initDefaultCommand(){

    }

    public DriveTrain(){
        this(true);
    }

    public DriveTrain(boolean setSensors){
        left_front = new TalonSRX(RobotMap.LEFT_FRONT);
        left_rear = new VictorSPX(RobotMap.LEFT_REAR);
        right_front = new TalonSRX(RobotMap.RIGHT_FRONT);
        right_rear = new VictorSPX(RobotMap.RIGHT_REAR);
        pigeon = new PigeonIMU(left_front);

        setSlaves();
        if(setSensors)
            setSensors();
    }

    void setSlaves(){
        left_rear.follow(left_front);
        right_rear.follow(right_front);
        slaveSet = true;
    }

    void setSensors(){
        if(!slaveSet)
            return;

        left_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX, RobotMap.TIMEOUT);
        left_front.setSensorPhase(leftPhase);

        right_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX, RobotMap.TIMEOUT);
        right_front.setSensorPhase(rightPhase);

        setPID(left_front);
        setPID(right_front);

        pidSet = true;
    }

    void setPID(TalonSRX talon){
        talon.config_kP(RobotMap.SLOT_INDEX, RobotMap.PGAIN, RobotMap.TIMEOUT);
        talon.config_kI(RobotMap.SLOT_INDEX, RobotMap.IGAIN, RobotMap.TIMEOUT);
        talon.config_kD(RobotMap.SLOT_INDEX, RobotMap.DGAIN, RobotMap.TIMEOUT);
        talon.config_kF(RobotMap.SLOT_INDEX, RobotMap.FGAIN, RobotMap.TIMEOUT);
    }

    public void setPeakSpeed(TalonSRX talon, double peak){
        
    }

    public void resetPosition(){
        left_front.setSelectedSensorPosition(0);
        right_front.setSelectedSensorPosition(0);
    }

    public void tankDrive(double left, double right){
        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }

    public void setPercent(double left, double right){
        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }

    public void setPercent(double percent){
        setPercent(percent, percent);
    }

    public double getAngle(){
        double[] data = new double[3];
        pigeon.getRawGyro(data);
        return pigeon.getFusedHeading();
    }

    public int getLeftPosition(){
        return left_front.getSelectedSensorPosition();
    }

    public int getRightPosition(){
        return right_front.getSelectedSensorPosition();
    }

    public void driveDistance(int inches){//Drive set distance on left and right
        driveDistance(inches, inches);
    }

    public void driveDistance(int inchesL, int inchesR){//Drive a left distance, drive a right distance
        status = DriverStatus.Distance;//Take over driver control
        left_front.set(ControlMode.Position, getLeftPosition()+inchesToNative(inchesL)); 
        right_front.set(ControlMode.Position, getRightPosition()+inchesToNative(inchesR));
    }

    //Native Units: 1 rotation / circumference = 4096 Native
    public double inchesToNative(int inches){
        return (inches/RobotMap.WHEEL_CIRCUMFERENCE)*RobotMap.NATIVE_UNITS;
    }

    public void locateLine(){//TODO: Use vision targets to locate the alignment line
        status = DriverStatus.Locate;
    }
    
    public boolean detectLine(){
        return lineC.get();
    }

    public String getLineBinary(){
        StringBuilder sb = new StringBuilder();
        sb.append(lineL.get() ? '1' : '0');
        sb.append(lineC.get() ? '1' : '0');
        sb.append(lineR.get() ? '1' : '0');
        return sb.toString();
    }

    //pre-condition found line at farthest end
    public void followLine(){//TODO: Account for drift
        double speed = Robot.input.getLineSpeed();
        switch(getLineBinary()){
            case "010"://We are centered
                setPercent(speed);
            break;
            case "100"://Left Only
                setPercent(speed/2, speed);
            break;
            case "001"://Right Only
                setPercent(speed, speed/2);
            break;
            case "000"://There is no line detected anymore
                setPercent(0);
                status = DriverStatus.Control;//Go back to driver controll
            break;
        }
    }
}