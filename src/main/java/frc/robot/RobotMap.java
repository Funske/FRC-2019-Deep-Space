package frc.robot;

import frc.robot.util.PIDData;

public class RobotMap{

    public static final boolean COMPETITON_MODE = false;

    //Motors
    public static final int LEFT_FRONT_RITA = 5, LEFT_REAR_RITA = 6, RIGHT_FRONT_RITA = 4, RIGHT_REAR_RITA = 3;
    public static final int LEFT_FRONT = 2, LEFT_REAR = 11, RIGHT_FRONT = 13, RIGHT_REAR = 14;
    public static final int ELEVATOR = 1, INTAKE = 12;

    //Solenoids
    public static final int GEAR_IN = 6, GEAR_OUT = 7;
    public static final int LIFT_IN = 0, LIFT_OUT = 1;
    public static final int GRAB_IN = 4, GRAB_OUT = 5;
    public static final int EXTEND_IN = 2, EXTEND_OUT = 3;

    //Input
    public static final double DEADBAND = 0.1;//Joystick deadband

    //PID
    public static final int TIMEOUT = 10;
    public static final PIDData DRIVEPID_HIGH = new PIDData(0.1, 0, 0, 0.3918, 0, TIMEOUT);
    public static final PIDData DRIVEPID_LOW = new PIDData(0.1, 0, 0, 0.12132, 1, TIMEOUT);
    public static final int ENCODER_UNITS = 4096;
    public static final int MAX_VELOCITY = 2000;//Per 100ms was 2000
    public static final int MAX_VELOCITY_HIGH = 3000;//Per 100ms
    public static final int MAX_INCH_PER_SECOND = 92;//in/s
    public static final double MAX_METERS_PER_SECOND = 2.682243;//Real: 2.682243
    public static final double MAX_RPM = 292.96875;
    public static final double TIMESTAMP = 0.02;

    public static final double VISION_P = 0.8, VISION_I = 0, VISION_D = 0;
    public static final double VISION_VELOCITY = MAX_METERS_PER_SECOND;//m/s
    public static final double VISION_ACCELERATION = 0.5;//m/s^2
    public static final double VISION_JERK = 60;//m/s^3
    
    //Field Information
    public static final double NEAR_ROCKET_ANGLE = 30;//Calibrated to test field
    public static final double FAR_ROCKET_ANGLE = 152;//Calibrated to test field
    public static final double CARGO_HOLE_ANGLE = 90;
    public static final double NEAR_ROCKET_ANGLE_RIGHT = -32;//Calibrated to test field
    public static final double FAR_ROCKET_ANGLE_RIGHT = -155;//Calibrated to test field
    public static final double CARGO_HOLE_ANGLE_RIGHT = -95;
    public static final double LOADING_STATION_ANGLE = 180;
    public static final double CARGO_SHIP_SIDE = 90;
    public static final double CARGO_SHIP_FRONT = 0;
    public static final int LEFT_DIR = 1;
    public static final int RIGHT_DIR = -1;

    //Elevator
    public static final int LEVEL0_POS = 0;//0
    public static final int LEVEL1_POS = 28672;//7  28,672
    public static final int LEVEL2_POS = 63023;//14
    public static final int LEVEL3_POS = 86200;//21.75
    public static final int LEVEL_CARGO = 45056;//11

    public static final int LEVEL0_POS_HATCH = 0;//2.5
    public static final int LEVEL1_POS_HATCH = 10240;//2.5
    public static final int LEVEL2_POS_HATCH = 43984;//10.25
    public static final int LEVEL3_POS_HATCH = 72500;//17.5

    //Robot Geometry
    public static final double WHEEL_DIAMETER = 3;//Inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    public static final double BASE_WIDTH = 26.5;//Inches
}