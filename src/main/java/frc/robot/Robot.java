/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriverStatus;

public class Robot extends TimedRobot {

  //Subsystems
  public static DriveTrain driveTrain = new DriveTrain(false);
  public static Input input = new Input();

  //Input
  boolean zeroed = true;//Have the controllers been zeroed?

  @Override
  public void robotInit() {

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

  @Override
  public void teleopPeriodic() {
    switch(driveTrain.status){
      case Control://We have driver control
        driveTrain.tankDrive(input.getXAxis(), input.getYAxis());

        if(driveTrain.detectLine()){
          driveTrain.status = DriverStatus.Follow;
          if(input.getXAxis() != 0 || input.getYAxis() != 0)
          zeroed = false;
        }
      break;
      case Distance:
      break;
      case Locate:
      break;
      case Follow://We are following a line
        if(zeroed && (input.getXAxis() != 0 || input.getYAxis() != 0))//We are trying to drive, overwrite line follower
          driveTrain.status = DriverStatus.Control;
        else if(!zeroed && input.getXAxis() == 0 && input.getYAxis() == 0)
          zeroed = true;

        driveTrain.followLine();
      break;
    }
  }

  @Override
  public void testPeriodic() {
  }
}
