package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;

public class RobotMath{

    public static double inchesToMeters(double inches){
        return inches * 0.0254;
    }

    public static double capSpeed(double speed){//Cap speed [-1, 1]
        double value = Math.min(Math.abs(speed), 1);//Caps the speed to +1
        return value * Math.signum(speed);//Returns capped speed * the direction of speed
    }

    public static void setPID(TalonSRX talon, PIDData data){
        talon.config_kP(data.getSlotIndex(), data.getP(), data.getTimeout());
        talon.config_kI(data.getSlotIndex(), data.getI(), data.getTimeout());
        talon.config_kD(data.getSlotIndex(), data.getD(), data.getTimeout());
        talon.config_kF(data.getSlotIndex(), data.getF(), data.getTimeout());
        talon.configAllowableClosedloopError(data.getSlotIndex(), data.getMaxError(), data.getTimeout());
    }

    public static void setPeakSpeed(TalonSRX talon, double peak){
        talon.configPeakOutputForward(peak);
        talon.configPeakOutputReverse(-peak);
    }
}