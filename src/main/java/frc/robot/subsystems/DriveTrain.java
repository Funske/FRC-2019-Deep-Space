package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.util.RobotMath;

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

    public boolean slaveSet = false, pidSet = false;
    public boolean highGear = true;

    //Sensor Phase
    private boolean left_phase = false, right_phase = false;
    //Inverted
    private InvertType left_inverted = InvertType.None, right_inverted = InvertType.InvertMotorOutput;

    //Speeds
    private double teleopSpeed = 1.0;
    private double timeToFullOpen = 0.25, timeToFullClosed = 0.25;
    private double currentSpeed = 1.0;
    private boolean forward_locked = false;
    public boolean is_angle_lock = false;
    public double angle_lock = 0;

    //Drive Train Status
    public static enum DriverStatus{Control, Vision};
    public DriverStatus status = DriverStatus.Control;

    public DoubleSolenoid gearChange = new DoubleSolenoid(RobotMap.GEAR_IN, RobotMap.GEAR_OUT);

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

        left_inverted = InvertType.InvertMotorOutput;
        right_inverted = InvertType.None;

        setSlaves();
        setMaxSpeed(currentSpeed);
        setSensors();
    }

    void setSlaves(){
        left_rear.follow(left_front);
        right_rear.follow(right_front);

        setBrakeMode(NeutralMode.Brake);

        left_front.configNeutralDeadband(0.05);
        left_rear.configNeutralDeadband(0.05);
        right_front.configNeutralDeadband(0.05);
        right_rear.configNeutralDeadband(0.05);

        left_front.setInverted(left_inverted);
        left_rear.setInverted(InvertType.FollowMaster);
        right_front.setInverted(right_inverted);
        right_rear.setInverted(InvertType.FollowMaster);

        RobotMath.setPeakSpeed(left_front, teleopSpeed);
        RobotMath.setPeakSpeed(right_front, teleopSpeed);

        slaveSet = true;
    }

    void setSensors(){
        if(!slaveSet)
            return;

        RobotMath.setPID(left_front, RobotMap.DRIVEPID_HIGH);
        RobotMath.setPID(right_front, RobotMap.DRIVEPID_HIGH);
        RobotMath.setPID(left_front, RobotMap.DRIVEPID_LOW);
        RobotMath.setPID(right_front, RobotMap.DRIVEPID_LOW);

        left_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.TIMEOUT);
        left_front.setSensorPhase(left_phase);

        right_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.TIMEOUT);
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

    public void setBrakeMode(NeutralMode mode){
        left_front.setNeutralMode(mode);
        left_rear.setNeutralMode(mode);
        right_front.setNeutralMode(mode);
        right_rear.setNeutralMode(mode);
    }

    public void setMaxSpeed(double speed){
        currentSpeed = speed;
        RobotMath.setPeakSpeed(left_front, speed);
        RobotMath.setPeakSpeed(left_rear, speed);
        RobotMath.setPeakSpeed(right_front, speed);
        RobotMath.setPeakSpeed(right_rear, speed);
    }

    public double getMaxSpeed(){
        return currentSpeed;
    }

    public void resetPosition(){
        left_front.setSelectedSensorPosition(0);
        right_front.setSelectedSensorPosition(0);
    }

    public void setStatus(DriverStatus status){
        this.status = status;
    }

    public DriverStatus getStatus(){
        return status;
    }

    public void tankDrive(double left, double right){
        double turn_throttle = 0;
        if(is_angle_lock){//Angle is locked
            turn_throttle = (angle_lock - getAngle()) * 0.004 - getGyroRate() * 0.0004;
        }

        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }

    public void tankDriveVelocity(double left, double right){
        if(forward_locked){
            right = left;
            double turn_throttle = (angle_lock - getAngle()) * 0.004 - (getGyroRate() * 0.0004);
            turn_throttle *= 0.3;

            left = RobotMath.capSpeed(left - turn_throttle);
            right = RobotMath.capSpeed(left + turn_throttle);
        }

        left_front.set(ControlMode.Velocity, left*getMaxVelocity());
        right_front.set(ControlMode.Velocity, right*getMaxVelocity());
    }

    public void setPercent(double left, double right){
        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }

    public void setPercent(double percent){
        setPercent(percent, percent);
    }

    public void toggleHighGear(){
        highGear = !highGear;
        if(highGear){
            gearChange.set(Value.kForward);
            System.out.println("High Gear");
            left_front.selectProfileSlot(RobotMap.DRIVEPID_HIGH.getSlotIndex(), 0);
            right_front.selectProfileSlot(RobotMap.DRIVEPID_HIGH.getSlotIndex(), 0);
        }else{
            gearChange.set(Value.kReverse);
            System.out.println("Low Gear");
            left_front.selectProfileSlot(RobotMap.DRIVEPID_LOW.getSlotIndex(), 0);
            right_front.selectProfileSlot(RobotMap.DRIVEPID_LOW.getSlotIndex(), 0);
        }
    }

    public void lowGear(){
        highGear = false;
        gearChange.set(Value.kReverse);
        System.out.println("Low Gear");
        left_front.selectProfileSlot(RobotMap.DRIVEPID_LOW.getSlotIndex(), 0);
        right_front.selectProfileSlot(RobotMap.DRIVEPID_LOW.getSlotIndex(), 0);
    }

    public void lockForward(){
        forward_locked = true;
    }

    public void unlockForward(){
        forward_locked = false;
    }

    public int getMaxVelocity(){
        if(highGear)
            return RobotMap.MAX_VELOCITY_HIGH;
        return RobotMap.MAX_VELOCITY;
    }

    public void testMaxVelocity(){
        resetGyro();
        int maxGyroAngle = 10;//Number of times to rotate
        double startTime = System.currentTimeMillis()*1000;//Seconds
        while(Math.abs(getAngle()) < 360*maxGyroAngle){
            setPercent(1, -1);
        }
        long endTime = System.currentTimeMillis()*1000;//Seconds
        System.out.println("[RESULTS] Start Time: " + (startTime) + " | End Time: " + (endTime) + " | Difference: " + (endTime-startTime));
    }

    public void testMaxVelocityLine(){
        setPercent(1.0, 1.0);
    }

    public void resetGyro(){
        pigeon.setYaw(0);
        pigeon.setFusedHeading(0);
    }

    public void setGyro(double angle){
        pigeon.setYaw(angle);
        pigeon.setFusedHeading(angle);
    }

    public double getAngle(){
        PigeonIMU.FusionStatus fusion = new PigeonIMU.FusionStatus();
        pigeon.getFusedHeading(fusion);
        return fusion.heading;
    }

    public double getAngle360(){//Returns and angle (-360, 360)
        double temp = getAngle();
        return (Math.abs(temp) % 180)*Math.signum(temp);
    }

    public double getGyroRate(){
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);
        return xyz[2];
    }

    public int getLeftPosition(){
        return left_front.getSelectedSensorPosition();
    }

    public int getRightPosition(){
        return right_front.getSelectedSensorPosition();
    }

    public double capSpeed(double speed){//Caps value from -1 to 1
        return Math.min(Math.abs(speed), 1);
    }

    //Native Units: 1 rotation / circumference = 4096 Native
    public double inchesToNative(int inches){
        return (inches/RobotMap.WHEEL_CIRCUMFERENCE)*RobotMap.ENCODER_UNITS;
    }
}