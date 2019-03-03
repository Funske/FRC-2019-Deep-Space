package frc.robot.input;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;

public class DriverProfile{

    public enum ControllerType{
        F310, Attack3, X3D
    }

    public enum ControlStyle{
        TankDrive, ArcadeDrive
    }

    public String driver_name = "The Real Tate Stanton";
    public ControllerType controller = ControllerType.Attack3;
    public ControlStyle style = ControlStyle.TankDrive;
    public int number_of_controllers = 2;
    public boolean has_rumble_feedback = false;

    public int left_reverse = 1;
    public int right_reverse = 1;

    public Joystick[] joysticks;
    public ArrayList<JoystickAction> actions = new ArrayList<JoystickAction>();

    public DriverProfile(String driver_name, ControllerType controller, ControlStyle style, int number_of_controllers, boolean has_rumble_feedback){
        this.driver_name = driver_name;
        this.controller = controller;
        this.style = style;
        this.number_of_controllers = number_of_controllers;
        this.has_rumble_feedback = has_rumble_feedback;

        joysticks = new Joystick[number_of_controllers];
        for(int i = 0; i < joysticks.length; i++){
            joysticks[i] = new Joystick(i);
        }
    }

    public void AddAction(JoystickAction action){
        actions.add(action);
    }
}