package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class ElevatorWidget extends SendableBase{

    private double encoderPosition = 0.0;
    private double targetLevel = 0;
    private String levelText = "";
    public ElevatorWidget(String levelText, double encoderPosition, double targetLevel){
        super(false);//Don't send to live window
        this.levelText = levelText;
        this.encoderPosition = encoderPosition;
        this.targetLevel = targetLevel;
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("encoderPosition", () -> encoderPosition, p -> {
            encoderPosition = p;
        });
        builder.addDoubleProperty("targetLevel", () -> targetLevel, e -> {
            targetLevel = e;
        });
        builder.addStringProperty("levelText", () -> levelText, null);
    }

    public int getTargetLevel(){
        return (int)targetLevel;
    }

    public void setEncoderPosition(double position){
        this.encoderPosition = position;
    }

    public double getEncoderPosition(){
        return encoderPosition;
    }

    public void setLevelText(String text){
        this.levelText = text;
    }
}