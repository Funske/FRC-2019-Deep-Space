/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.input.Input;
import frc.robot.shuffleboard.ElevatorWidget;
import frc.robot.shuffleboard.VisionTargetWidget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.DriveTrain.DriverStatus;
import frc.robot.util.AccelTracker;
import frc.robot.util.GyroPID;
import frc.robot.util.RobotMath;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends TimedRobot {

  //Subsystems
  public static DriveTrain driveTrain = new DriveTrain(true);
  public static Lift lift = new Lift();
  public static Elevator elevator = new Elevator();
  public static Intake intake = new Intake();
  public static Input input = new Input();
  Compressor comp = new Compressor();

  //Input
  private boolean zeroed = false;//Have the controllers been zeroed?
  public int target_level = 1;

  //Vision
  private boolean vision_init = false;
  public double forward_throttle = 0.5, max_turn = 0.3;//%
  public double vision_tolerance = 0.5;//Degrees
  public GyroPID gyro_align;
  private EncoderFollower vision_left, vision_right;
  private boolean followTragectory = false;

  private int vision_updateCount = 0, vision_period = 25;
  private boolean vision_updated = false;

  //Dashboard
  public ShuffleboardTab tab = Shuffleboard.getTab("Deep Space");

  //public NetworkTableEntry network_maxTurn = Shuffleboard.getTab("Vision").add("Max Turn", max_turn).getEntry();
  //public NetworkTableEntry network_maxForward = Shuffleboard.getTab("Vision").add("Max Forward", forward_throttle).getEntry();
  public NetworkTableEntry robot_angle = Shuffleboard.getTab("SmartDashboard").add("Robot Angle", driveTrain.getAngle()).withPosition(7, 1).getEntry();
  public NetworkTableEntry left_pos = Shuffleboard.getTab("SmartDashboard").add("Left Pos", driveTrain.getLeftPosition()).withPosition(7, 4).getEntry();
  public NetworkTableEntry right_pos = Shuffleboard.getTab("SmartDashboard").add("Right Pos", driveTrain.getRightPosition()).withPosition(8, 4).getEntry();
  public NetworkTableEntry calc_angle = Shuffleboard.getTab("SmartDashboard").add("Calc Angle", 0).withPosition(7, 2).getEntry();
  public NetworkTableEntry adjusted_hor = Shuffleboard.getTab("SmartDashboard").add("Adjusted", 0).withPosition(7, 3).getEntry();

  public NetworkTableEntry target_angle;

  //Rocket Near: 0 | Rocket Cargo: 1 | Rocket Far: 2 | Cargo Left: 3 | Cargo Front: 4 | Cargo Right: 5 | Rocket Near: 6
  //Rocket Cargo: 7 | Rocket Far: 8 | Loading Sation: 9
  public int field_target = 0;

  @Override
  public void robotInit() {
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setResolution(320, 180);
    cam.setFPS(30);
    elevator.resetPosition();
    tab.add("Elevator Info", elevator.getWidget());
    tab.add("Compressor Current", comp.getCompressorCurrent());
    target_angle = tab.add("Target Angle", RobotMap.NEAR_ROCKET_ANGLE).getEntry();
    //tab.add("Vision Info", visionWidget);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    driveTrain.setBrakeMode(NeutralMode.Coast);
    //elevator.setCoast();
  }

  int addative = 1;
  @Override
  public void disabledPeriodic() {
    //addative *= -1;
    double target_yaw = SmartDashboard.getNumber("target_yaw", 0);//Radians
    double target_distance = SmartDashboard.getNumber("target_distance", 0);//In inches
    robot_angle.forceSetDouble(driveTrain.getAngle360());
    double calculatedAngle = (driveTrain.getAngle360()-Math.toDegrees(target_yaw))-32;//Degrees
    calculatedAngle = (driveTrain.getAngle360()+Math.toDegrees(target_yaw))-32;
    //calculatedAngle = -Math.toDegrees(target_yaw)-32;
    double end_x = Math.abs(target_distance*Math.cos(Math.toRadians(calculatedAngle)));//Meters
    double end_y = Math.abs(target_distance*Math.sin(Math.toRadians(calculatedAngle)));//Meters
    calc_angle.forceSetDouble(calculatedAngle);
    left_pos.forceSetDouble(end_x);
    right_pos.forceSetDouble(end_y);
    adjusted_hor.forceSetDouble(end_y*Math.cos(Math.toRadians(driveTrain.getAngle360())));
    //left_pos.forceSetDouble(driveTrain.getLeftPosition()+addative);
    //right_pos.forceSetDouble(driveTrain.getRightPosition()+addative);

  }

  @Override
  public void autonomousInit() {
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  public void teleopInit() {
    zeroed = true;
    vision_init = false;
    driveTrain.status = DriverStatus.Control;
    driveTrain.setBrakeMode(NeutralMode.Brake);
    driveTrain.resetGyro();
    driveTrain.resetPosition();
    driveTrain.lowGear();
    elevator.setLevel(0);

    //forward_throttle = network_maxForward.getDouble(0.5);
    //max_turn = network_maxTurn.getDouble(0.5);
    gyro_align = new GyroPID(max_turn);

    //VisionInit();
  }

  int lineCount = 0;
  @Override
  public void teleopPeriodic() {
    //int tape_detected = (int)SmartDashboard.getNumber("target_identified", 0);
    //robot_angle.forceSetDouble(driveTrain.getAngle());

    switch(driveTrain.status){
      case Control://We have driver control
        input.InputPeriodic();
        input.rumblePeriodic();
        ControlPeriodic();
        if(vision_init)     
          vision_init = false;   
      break;
      case Vision:
        if(!vision_init){
          VisionInit();
          vision_init = true;
        }

        VisionPeriodic();

        /*if(tape_detected == 0)//We lost the vision tape
          driveTrain.status = DriverStatus.Control;*/
      break;
    }

    elevator.updateWidget();
  }

  public void ControlPeriodic(){
    endPoint = target_angle.getDouble(RobotMap.NEAR_ROCKET_ANGLE);
    angleDifference.forceSetDouble(driveTrain.getAngle360());
    if(elevator.getLevel() == 3 && elevator.getPosition() > 80000 && elevator.getVelocity() < 300 && elevator.getCargoMode()){
      try{
        Thread.sleep(100);//0.1s seconds
      }catch(InterruptedException e){
        System.out.println("Cargo Outtake Sleep Caught");
      }

      intake.setCargoMotor(-intake.cargoSpeed);//Outtake

      try{
        Thread.sleep(1000);//1s seconds
      }catch(InterruptedException e){
        System.out.println("Cargo Outtake Sleep Caught");
      }

      intake.setCargoMotor(0);
      elevator.setLevel(2);
    }
    if(elevator.getLevel() == 5 && elevator.getPosition() > 47000 && elevator.getPosition() < 50000){
      intake.setCargoMotor(intake.cargoSpeed);
    }
  }

  public void VisionInit(){
    zeroed = false;
    followTragectory = false;
  }

  double endPoint = RobotMap.NEAR_ROCKET_ANGLE;
  double rotate_end = 0;
  public void VisionPeriodic(){
    double target_yaw = SmartDashboard.getNumber("target_yaw", 0);
    double target_distance = SmartDashboard.getNumber("target_distance", 0);
    double robot_angle = driveTrain.getAngle360();
    angleDifference.forceSetDouble(robot_angle);
    /*if(rotate_end == 0){//Can't travel in a straight line
      rotate_end = robot_angle+Math.toDegrees(target_yaw);
    }*/

    if(!followTragectory){//Current robot position casues inaccurate tragectory, fix that
      
      /*gyro_align.setSetpoint(rotate_end);
      double turn_throttle = gyro_align.pidUpdate();

      driveTrain.tankDriveVelocity(-turn_throttle*RobotMap.MAX_VELOCITY, turn_throttle*RobotMap.MAX_VELOCITY);

      if(Math.abs(robot_angle-rotate_end) <= vision_tolerance){//End condition
        System.out.println("End Condition Met");
        driveTrain.setPercent(0);

        try{
          Thread.sleep(200);//Pause for 0.2s to get system to rest
        }catch(InterruptedException e){
          System.out.println("Vision Sleep Caught");
        }

        
      }*/

      GenerateTragectory(RobotMath.inchesToMeters(target_distance), robot_angle, endPoint, target_yaw, RobotMap.LEFT_DIR, true);
      followTragectory = true;

    }else//We have generated an accurate tragectory, now follow it
      FollowTragectory();
  }

  public void FollowPeriodic(){
    
  }

  NetworkTableEntry angleDifference = Shuffleboard.getTab("SmartDashboard").add("Angle Difference", 0).getEntry();
  NetworkTableEntry desiredHeading = Shuffleboard.getTab("SmartDashboard").add("Desired Heading", 0).getEntry();

  NetworkTableEntry left_enc = Shuffleboard.getTab("SmartDashboard").add("Left Encoder", 0).getEntry();
  NetworkTableEntry right_enc = Shuffleboard.getTab("SmartDashboard").add("Right Encoder", 0).getEntry();
  private double angle_adjusment = 0;
  private void FollowTragectory(){
    double left_speed = vision_left.calculate(driveTrain.getLeftPosition());
    double right_speed = vision_right.calculate(driveTrain.getRightPosition());

    left_enc.forceSetDouble(driveTrain.getLeftPosition());
    right_enc.forceSetDouble(driveTrain.getRightPosition());

    double gyro_heading = driveTrain.getAngle360()-angle_adjusment;
    double desired_heading = Pathfinder.r2d(vision_left.getHeading());
    //if(desired_heading > 180)//Wrap desired heading to [-180, 180]
      //desired_heading -= 360;
    desired_heading = Pathfinder.boundHalfDegrees(desired_heading);
    double angle_diference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    //double angle_diference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    /*
    robot_angle.forceSetDouble(gyro_heading);
    desiredHeading.forceSetDouble(desired_heading);*/

    double turn_throttle = 0.8 * (1.0/80.0) * angle_diference;
    SmartDashboard.putNumber("Vision Left Speed", left_speed);
    SmartDashboard.putNumber("Vision Right Speed", right_speed);
    SmartDashboard.putNumber("Turn Throttle", turn_throttle);

    driveTrain.setPercent(-RobotMath.capSpeed(left_speed - turn_throttle), -RobotMath.capSpeed(right_speed + turn_throttle));

    if(vision_left.isFinished() && vision_right.isFinished()){
      driveTrain.status = DriverStatus.Control;
    }
  }

  public void GenerateTragectory(double target_distance, double robot_angle, double end_angle, double target_yaw, int dir){
    GenerateTragectory(target_distance, robot_angle, end_angle, target_yaw, dir, false);
  }

  public void GenerateTragectory(double target_distance, double robot_angle, double end_angle, double target_yaw, int dir, boolean verbose){
    driveTrain.resetPosition();

    try{
      Thread.sleep(100);
    }catch(InterruptedException e){
      System.out.println("Vision Sleep Caught");
    }

    /*double calculated_angle = Math.toRadians((robot_angle+Math.toDegrees(target_yaw))-end_angle);//Radians
    if(calculated_angle < 0)
      dir = RobotMap.LEFT_DIR;
    else
      dir = RobotMap.RIGHT_DIR;

    double end_x = Math.abs(target_distance * Math.cos(calculated_angle));//Always drive forward
    double end_y = Math.abs(target_distance * Math.sin(calculated_angle)) * dir;//Drive left or right depending on dir*/

    double calculated_angle = (end_angle-driveTrain.getAngle360())+Math.toDegrees(target_yaw);
    double end_x = Math.abs(target_distance*Math.cos(Math.toRadians(calculated_angle)))-RobotMath.inchesToMeters(14);//Meters
    double end_y = Math.abs(target_distance*Math.sin(Math.toRadians(calculated_angle)))-RobotMath.inchesToMeters(10);//Meters
    end_y *= Math.signum(target_yaw);
    //if(end_angle != 0)
      //end_y = 0;

    angle_adjusment = robot_angle;
    Waypoint[] points = {new Waypoint(0, 0, 0), new Waypoint(end_x, end_y, Math.toRadians(end_angle-robot_angle))};
    //Waypoint[] points = {new Waypoint(0, 0, 0), new Waypoint(1.0, 0, Math.toRadians(45))};
    Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 
    RobotMap.TIMESTAMP, RobotMap.VISION_VELOCITY, RobotMap.VISION_ACCELERATION, RobotMap.VISION_JERK);
    Trajectory trajectory = Pathfinder.generate(points, config);
    TankModifier vision_path = new TankModifier(trajectory).modify(RobotMath.inchesToMeters(RobotMap.BASE_WIDTH));

    vision_left = new EncoderFollower(vision_path.getLeftTrajectory());
    vision_right = new EncoderFollower(vision_path.getRightTrajectory());

    //Left Encoder
    vision_left.configureEncoder(driveTrain.getLeftPosition(), RobotMap.ENCODER_UNITS, RobotMath.inchesToMeters(RobotMap.WHEEL_DIAMETER));
    vision_left.configurePIDVA(RobotMap.VISION_P, RobotMap.VISION_I, RobotMap.VISION_D, 1/RobotMap.VISION_VELOCITY, 0);

    //Right Encoder
    vision_right.configureEncoder(driveTrain.getRightPosition(), RobotMap.ENCODER_UNITS, RobotMath.inchesToMeters(RobotMap.WHEEL_DIAMETER));
    vision_right.configurePIDVA(RobotMap.VISION_P, RobotMap.VISION_I, RobotMap.VISION_D, 1/RobotMap.VISION_VELOCITY, 0);

    File main = new File("/home/lvuser/mainpath.csv");
    Pathfinder.writeToCSV(main, trajectory);

    if(verbose){
      System.out.println("Robot Angle: " + robot_angle);
      System.out.println("End Angle: " + end_angle);
      System.out.println("Calculated Angle: " + calculated_angle);
      System.out.println("Distance(m): " + target_distance);
      System.out.println("(" + end_x + ", " + end_y + ")");
    }
  }

  @Override
  public void testInit() {
    driveTrain.testMaxVelocity();
  }

  @Override
  public void testPeriodic() {
  }

  public double getTargetAngle(){
    switch(field_target){
      case 0://Rocket Near (Left)
        return RobotMap.NEAR_ROCKET_ANGLE;
      case 1://Rocket Cargo (Left)
        return RobotMap.CARGO_HOLE_ANGLE;
      case 2://Rocket Far (Left)
        return RobotMap.FAR_ROCKET_ANGLE;
      case 3://Ship Left
        return RobotMap.CARGO_SHIP_SIDE;
      case 4://Ship Front
        return RobotMap.CARGO_SHIP_FRONT;
      case 5://Ship Right
        return -RobotMap.CARGO_SHIP_SIDE;
      case 6://Rocket Near (Right)
        return -RobotMap.NEAR_ROCKET_ANGLE;
      case 7://Rocket Cargo (Right)
        return -RobotMap.CARGO_HOLE_ANGLE;
      case 8://Rocket Far (Right)
        return -RobotMap.FAR_ROCKET_ANGLE;
      case 9://Loading Station
        return RobotMap.LOADING_STATION_ANGLE;
    }
    return 0;
  }
}
