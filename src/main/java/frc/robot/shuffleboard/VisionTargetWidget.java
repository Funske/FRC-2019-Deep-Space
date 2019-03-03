package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class VisionTargetWidget extends SendableBase{

    private final double VISION_WIDTH = 416;
    private final double WIDGET_WIDTH = 256;
    private final double RATIO = WIDGET_WIDTH/VISION_WIDTH;

    private double centerLine, targetDistance, targetHeight;

    public VisionTargetWidget(double centerLine, double targetDistance, double targetHeight){
        this.centerLine = setCenterLine(centerLine);
        this.targetDistance = setTargetDistance(targetDistance);
        this.targetHeight = setTargetHeight(targetHeight);
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("VisionTarget");
        builder.addDoubleProperty("centerLine", () -> centerLine, null);
        builder.addDoubleProperty("targetDistance", () -> targetDistance, null);
        builder.addDoubleProperty("targetHeight", () -> targetHeight, null);
    }

    public double setCenterLine(double centerLine){
        return centerLine * RATIO;
    }

    public double setTargetDistance(double distance){
        return distance * RATIO;
    }

    public double setTargetHeight(double height){
        return height * RATIO;
    }
}