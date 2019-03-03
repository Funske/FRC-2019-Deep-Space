package frc.robot.input;

public class JoystickAction{

    public enum EventType{
        Axis, ButtonPress, ButtonHold, ButtonRelease, POV;
    }

    public String action_name = "";
    public int controller_number = 1;//What controller to use
    public EventType action_type = EventType.Axis;
    public int joystick_number = 0;
    public double deadband = 0;
    public boolean isPressed = false;
    public JoystickAction secondary;

    public JoystickAction(String action_name, int controller_number, int joystick_number, EventType action_type){
        this.action_name = action_name;
        this.controller_number = controller_number;
        this.joystick_number = joystick_number;
        this.action_type = action_type;
    }

    public JoystickAction(String action_name, int controller_number, int joystick_number, EventType action_type, double deadband){
        this.action_name = action_name;
        this.controller_number = controller_number;
        this.joystick_number = joystick_number;
        this.action_type = action_type;
        this.deadband = deadband;
    }

    public void setSecondaryAction(JoystickAction secondary){
        this.secondary = secondary;
    }
}