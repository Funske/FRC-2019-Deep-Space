/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriverStatus;
import frc.robot.util.GyroSource;
import frc.robot.util.VisionOutput;

public class Robot extends TimedRobot {

  //Subsystems
  public static DriveTrain driveTrain = new DriveTrain(true);
  public static Input input = new Input();

  //Input
  private boolean zeroed = true;//Have the controllers been zeroed?

  //Vision
  public double forward_throttle = 0.5, vision_tolerance = 2.0;
  public double max_turn = 0.3;
  private VisionOutput vision_output = new VisionOutput();
  private GyroSource pid_gyro = new GyroSource();
  private PIDController vision_align;
  public double vision_gain = 0.1;
  public NetworkTableEntry network_maxTurn = Shuffleboard.getTab("Vision").add("Max Turn", max_turn).getEntry();
  public NetworkTableEntry network_maxForward = Shuffleboard.getTab("Vision").add("Max Forward", forward_throttle).getEntry();

  @Override
  public void robotInit() {
    vision_align = new PIDController(vision_gain, 0, 0, pid_gyro, vision_output);
    vision_align.setInputRange(-180.0, 180.0);
    vision_align.setOutputRange(-max_turn, max_turn);
    vision_align.setAbsoluteTolerance(vision_tolerance);
    vision_align.setContinuous(true);

    Shuffleboard.getTab("Vision").add("Vision PID", vision_align);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  public void teleopInit() {
    zeroed = true;
    driveTrain.status = DriverStatus.Control;
    driveTrain.resetGyro();
    //VisionInit();

    forward_throttle = network_maxForward.getDouble(0.5);
    max_turn = network_maxTurn.getDouble(0.5);
    vision_align.setOutputRange(-max_turn, max_turn);
  }

  int lineCount = 0;
  @Override
  public void teleopPeriodic() {
    boolean tape_detected = true;

    /*lineCount++;
    if(lineCount >= 10){
      System.out.println("Drive Train Status: " + driveTrain.status);
      lineCount = 0;
    }*/

    switch(driveTrain.status){
      case Control://We have driver control
        ControlPeriodic();

      //if(tape_detected && input.getVisionButton()){//Enable Vision Control
        //VisionInit();
        //driveTrain.status = DriverStatus.Vision;
      //}
      break;
      case Distance:
        DistancePeriodic();
      break;
      case Vision:
        VisionPeriodic();

      if(!tape_detected)//We lost the vision tape
        driveTrain.status = DriverStatus.Control;
      break;
      case Follow://We are following a line
        FollowPeriodic();
      break;
    }
  }

  public void ControlPeriodic(){
    driveTrain.setPercent(1.0, 0);

    //driveTrain.tankDrive(input.getXAxis(), -input.getYAxis());
    //if(input.getXAxis() == 0 && input.getYAxis() == 0)
      //driveTrain.setPercent(0);

    /*lineCount++;
    if(lineCount >= 10){
      System.out.println("Control: " + driveTrain.getLineBinary());
      lineCount = 0;
    }
    if(driveTrain.detectLine()){
      driveTrain.status = DriverStatus.Follow;
      if(input.getXAxis() != 0 || input.getYAxis() != 0)
        zeroed = false;
    }*/
  }

  public void DistancePeriodic(){

  }

  public void VisionInit(){
    pid_gyro.setReset(Robot.driveTrain.getAngle());
  }

  public void VisionPeriodic(){
    double target_yaw = Math.toDegrees(SmartDashboard.getNumber("target_yaw", 0));
    double turn_throttle = 0;
    //Negative = more to the right
    //Positive = more to the left

    visionPID(target_yaw);
    turn_throttle = vision_output.getOutput()*-1;

    if(Math.abs(target_yaw) <= vision_tolerance)
      turn_throttle = 0;

    double left_speed = (forward_throttle-turn_throttle)/2.5;
    double right_speed = (forward_throttle+turn_throttle)/2.5;

    lineCount++;
    if(lineCount >= 25){
      System.out.println("Target Yaw: " + target_yaw);
      System.out.println("Robot Angle: " + driveTrain.getAngle() + " | Reset: " + pid_gyro.getReset());
      System.out.println("R Speed: " + right_speed + " | Turn: " + turn_throttle);
      System.out.println("L Speed: " + left_speed);
      lineCount = 0;
    }
    driveTrain.tankDriveVelocity(left_speed*RobotMap.MAX_VELOCITY, right_speed*-RobotMap.MAX_VELOCITY);
    
  }

  void visionPID(double setpoint){
    if(Math.abs(setpoint) <= vision_tolerance)
      setpoint = 0;

    vision_align.setSetpoint(setpoint);
    vision_align.enable();
  }

  public void FollowPeriodic(){
    if(zeroed && (input.getXAxis() != 0 || input.getYAxis() != 0)){//We are trying to drive, overwrite line follower
      //driveTrain.status = DriverStatus.Control;
      //System.out.println("Zeroed and moving");
    }else if(!zeroed && input.getXAxis() == 0 && input.getYAxis() == 0)
      zeroed = true;

    driveTrain.followLine();
  }

  @Override
  public void testPeriodic() {
  }
}
