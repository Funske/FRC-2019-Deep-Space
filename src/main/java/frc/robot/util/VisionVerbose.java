package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class VisionVerbose{

    ShuffleboardTab verbose = Shuffleboard.getTab("Verbose");

    public NetworkTableEntry robot_angle = verbose.add("Robot Angle", Robot.driveTrain.getAngle()).withPosition(7, 1).getEntry();
    public NetworkTableEntry left_pos = verbose.add("Left Pos", Robot.driveTrain.getLeftPosition()).withPosition(7, 4).getEntry();
    public NetworkTableEntry right_pos = verbose.add("Right Pos", Robot.driveTrain.getRightPosition()).withPosition(8, 4).getEntry();
    public NetworkTableEntry calc_angle = verbose.add("Calc Angle", 0).withPosition(7, 2).getEntry();
    public NetworkTableEntry adjusted_hor = verbose.add("Adjusted", 0).withPosition(7, 3).getEntry();

    int addative = 1;
    public void verboseLog(){
        addative *= -1;
        double target_yaw = SmartDashboard.getNumber("target_yaw", 0);//Radians
        double target_distance = SmartDashboard.getNumber("target_distance", 0);//In inches
        robot_angle.forceSetDouble(Robot.driveTrain.getAngle360());
        double calculatedAngle = (Robot.driveTrain.getAngle360()-Math.toDegrees(target_yaw))-32;//Degrees
        calculatedAngle = (Robot.driveTrain.getAngle360()+Math.toDegrees(target_yaw))-32;
        //calculatedAngle = -Math.toDegrees(target_yaw)-32;
        double end_x = Math.abs(target_distance*Math.cos(Math.toRadians(calculatedAngle)));//Meters
        double end_y = Math.abs(target_distance*Math.sin(Math.toRadians(calculatedAngle)));//Meters
        calc_angle.forceSetDouble(calculatedAngle);
        //left_pos.forceSetDouble(end_x);
        //right_pos.forceSetDouble(end_y);
        //adjusted_hor.forceSetDouble(end_y*Math.cos(Math.toRadians(Robot.driveTrain.getAngle360())));
        //left_pos.forceSetDouble(driveTrain.getLeftPosition()+addative);
        //right_pos.forceSetDouble(driveTrain.getRightPosition()+addative);

        addative *= -1;
        left_pos.forceSetDouble(Robot.driveTrain.getLeftPosition()+addative);
        right_pos.forceSetDouble(Robot.driveTrain.getRightPosition()+addative);
        adjusted_hor.forceSetDouble(Robot.elevator.getPosition()+addative);
    }
}