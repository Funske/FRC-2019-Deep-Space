package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class DriveTrain extends Subsystem{

    TalonSRX left_front, right_front;
    TalonSRX left_rear, right_rear;
    public PigeonIMU pigeon;
    DigitalInput lineC = new DigitalInput(RobotMap.CENTER_FOLLOWER), 
    lineL = new DigitalInput(RobotMap.LEFT_FOLLOWER), 
    lineR = new DigitalInput(RobotMap.RIGHT_FOLLOWER);

    public boolean slaveSet = false, pidSet = false;
    public boolean highGear = true;
    //Sensor Phase
    private boolean left_phase = true, right_phase = true;
    //Inverted
    private InvertType left_inverted = InvertType.None, right_inverted = InvertType.InvertMotorOutput;

    public double teleopSpeed = 1.0, autoSpeed = 0.5;
    public double timeToFullOpen = 1.0, timeToFullClosed = 0.05;

    public static enum DriverStatus{Control, Distance, Follow, Vision};
    public DriverStatus status = DriverStatus.Control;

    public double timestamp = 0.2;//Seconds between motion calculations

    public void initDefaultCommand(){

    }

    public DriveTrain(){
        this(true);
    }

    public DriveTrain(boolean setSensors){
        left_front = new TalonSRX(RobotMap.LEFT_FRONT);
        left_rear = new TalonSRX(RobotMap.LEFT_REAR);
        right_front = new TalonSRX(RobotMap.RIGHT_FRONT);
        right_rear = new TalonSRX(RobotMap.RIGHT_REAR);
        pigeon = new PigeonIMU(right_rear);

        setSlaves();
        if(setSensors)
            setSensors();
    }

    void setSlaves(){
        left_rear.follow(left_front);
        right_rear.follow(right_front);

        left_front.setNeutralMode(NeutralMode.Brake);
        left_rear.setNeutralMode(NeutralMode.Brake);
        right_front.setNeutralMode(NeutralMode.Brake);
        right_rear.setNeutralMode(NeutralMode.Brake);

        left_front.configNeutralDeadband(0.05);
        left_rear.configNeutralDeadband(0.05);
        right_front.configNeutralDeadband(0.05);
        right_rear.configNeutralDeadband(0.05);

        left_front.setInverted(left_inverted);
        left_rear.setInverted(InvertType.FollowMaster);
        right_front.setInverted(right_inverted);
        right_rear.setInverted(InvertType.FollowMaster);

        setPeakSpeed(left_front, teleopSpeed);
        setPeakSpeed(right_front, teleopSpeed);

        slaveSet = true;
    }

    void setSensors(){
        if(!slaveSet)
            return;

        setPID(left_front);
        setPID(right_front);

        left_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX, RobotMap.TIMEOUT);
        left_front.setSensorPhase(left_phase);

        right_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX, RobotMap.TIMEOUT);
        right_front.setSensorPhase(right_phase);

        left_front.configOpenloopRamp(timeToFullOpen);
        right_front.configOpenloopRamp(timeToFullOpen);
        left_front.configClosedloopRamp(timeToFullClosed);
        right_front.configClosedloopRamp(timeToFullClosed);

        left_front.configMotionCruiseVelocity(RobotMap.MAX_VELOCITY/2);
        left_front.configMotionAcceleration(RobotMap.MAX_VELOCITY/2);
        right_front.configMotionCruiseVelocity(RobotMap.MAX_VELOCITY/2);
        right_front.configMotionAcceleration(RobotMap.MAX_VELOCITY/2);

        pidSet = true;
    }

    void setPID(TalonSRX talon){
        talon.config_kP(RobotMap.SLOT_INDEX, RobotMap.PGAIN, RobotMap.TIMEOUT);
        talon.config_kI(RobotMap.SLOT_INDEX, RobotMap.IGAIN, RobotMap.TIMEOUT);
        talon.config_kD(RobotMap.SLOT_INDEX, RobotMap.DGAIN, RobotMap.TIMEOUT);
        talon.config_kF(RobotMap.SLOT_INDEX, RobotMap.FGAIN, RobotMap.TIMEOUT);
    }

    public void setPeakSpeed(TalonSRX talon, double peak){
        talon.configPeakOutputForward(peak);
        talon.configPeakOutputReverse(-peak);
    }

    public void resetPosition(){
        left_front.setSelectedSensorPosition(0);
        right_front.setSelectedSensorPosition(0);
    }

    public void tankDrive(double left, double right){
        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }

    int count = 0;
    public void tankDriveVelocity(double left, double right){
        left_front.set(ControlMode.Velocity, left);
        right_front.set(ControlMode.Velocity, right);
    }

    public void setPercent(double left, double right){
        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }

    public void setPercent(double percent){
        setPercent(percent, percent);
    }

    public void resetGyro(){
        pigeon.setFusedHeading(0);
    }

    public double getAngle(){
        //double[] data = new double[3];
        //pigeon.getRawGyro(data);
        PigeonIMU.FusionStatus fusion = new PigeonIMU.FusionStatus();
        pigeon.getFusedHeading(fusion);
        return fusion.heading;
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

    public double capSpeed(double speed){//Caps value from -1 to 1
        return Math.min(Math.abs(speed), 1);
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
    
    public boolean detectLine(){
        String line = getLineBinary();
        return line.charAt(0) == '1' || line.charAt(1) == '1' || line.charAt(2) == '1';
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
                setPercent(speed, -speed);
            break;
            case "100"://Left Only
                setPercent(-speed/2, -speed);
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