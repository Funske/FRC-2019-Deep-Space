package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class FieldTargetWidget extends SendableBase{

    private double target = 0, angle = 0;

    public FieldTargetWidget(int target, double angle){
        this.target = target;
        this.angle = angle;
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("FieldTarget");
        builder.addDoubleProperty("target", () -> target, t -> {
            this.target = t;
        });
        builder.addDoubleProperty("angle", () -> angle, null);
    }

    public void setAngle(double angle){
        this.angle = angle;
    }

    public int getTarget(){
        return (int)target;
    }
}