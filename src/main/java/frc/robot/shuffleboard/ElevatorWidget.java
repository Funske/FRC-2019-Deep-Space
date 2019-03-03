package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class ElevatorWidget extends SendableBase{

    private double encoderPosition = 0.0;
    private String levelText = "";
    public ElevatorWidget(String levelText, double encoderPosition){
        super(false);//Don't send to live window
        this.levelText = levelText;
        this.encoderPosition = encoderPosition;
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("encoderPosition", () -> encoderPosition, null);
        builder.addStringProperty("levelText", () -> levelText, null);
    }

    public void setEncoderPosition(double position){
        this.encoderPosition = position;
    }

    public void setLevelText(String text){
        this.levelText = text;
    }
}