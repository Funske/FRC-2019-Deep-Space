package frc.robot.input;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.input.DriverProfile.ControlStyle;
import frc.robot.input.DriverProfile.ControllerType;
import frc.robot.input.JoystickAction.EventType;
import frc.robot.subsystems.DriveTrain.DriverStatus;

public class Input{

    public int driver_index = 0;
    public ArrayList<DriverProfile> profiles = new ArrayList<DriverProfile>();
    private DriverProfile current_profile;

    private boolean elevatorManualMode = false;
    public boolean intakeManualMode = false;

    public Input(){
        CreateTroy310Profile();
        current_profile = profiles.get(driver_index);
    }

    public void InputPeriodic(){
        boolean cargoMoving = false;
        boolean elevatorMove = false;

        for(JoystickAction action : current_profile.actions){
            switch(action.action_type){
                case Axis:
                //Event: "move_open"
                if(action.action_name.equals("move_open")){
                    double left = 0, right = 0;
                    if(Math.abs(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number))-action.deadband > 0)
                        left = current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number);
                    if(Math.abs(current_profile.joysticks[action.secondary.controller_number].getRawAxis(action.secondary.joystick_number))-action.secondary.deadband > 0)
                        right = current_profile.joysticks[action.secondary.controller_number].getRawAxis(action.secondary.joystick_number);
                    
                    if(!elevatorManualMode)
                        Robot.driveTrain.tankDrive(left*current_profile.left_reverse, right*current_profile.right_reverse);
                    else
                        Robot.elevator.setPercent(left);
                }
                //Event: "move_closed"
                if(action.action_name.equals("move_closed")){
                    double left = 0, right = 0;
                    if(Math.abs(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number))-action.deadband > 0)
                        left = current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number);
                    if(Math.abs(current_profile.joysticks[action.secondary.controller_number].getRawAxis(action.secondary.joystick_number))-action.secondary.deadband > 0)
                        right = current_profile.joysticks[action.secondary.controller_number].getRawAxis(action.secondary.joystick_number);
                    Robot.driveTrain.tankDriveVelocity(left*current_profile.left_reverse, right*current_profile.right_reverse);
                }
                //Event: "move_sideways"
                if(action.action_name.equals("move_sideways_closed")){
                    double forward = 0, turn = 0;
                    if(Math.abs(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number))-action.deadband > 0)
                        forward = current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number);
                    if(Math.abs(current_profile.joysticks[action.secondary.controller_number].getRawAxis(action.secondary.joystick_number))-action.secondary.deadband > 0)
                        turn = current_profile.joysticks[action.secondary.controller_number].getRawAxis(action.secondary.joystick_number);
                    if(!elevatorManualMode){
                        Robot.driveTrain.tankDriveVelocity((forward-turn), (forward+turn));
                    }else{
                        Robot.elevator.setPercent(forward);
                    }
                }
                //Event: "intake_cargo_axis"
                if(action.action_name.equals("intake_cargo_axis")){
                    if(Math.abs(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number))-action.deadband > 0 && !intakeManualMode){
                        Robot.intake.setCargoMotor(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number));
                        cargoMoving = true;
                    }
                }
                //Event: "outtake_cargo_axis"
                if(action.action_name.equals("outtake_cargo_axis")){
                    if(Math.abs(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number))-action.deadband > 0  && !intakeManualMode){
                        Robot.intake.setCargoMotor(-current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number));
                        cargoMoving = true;
                    }
                }
                //Event: "elevator_axis"
                if(action.action_name.equals("elevator_axis")){
                    if(Math.abs(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number))-action.deadband > 0)
                        Robot.elevator.setPercent(current_profile.joysticks[action.controller_number].getRawAxis(action.joystick_number));
                }

                break;
                case ButtonPress:
                //Event: "elevator_increase_button"
                if(action.action_name.equals("elevator_increase_button")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        Robot.elevator.increaseLevel();
                }
                //Event: "elevator_decrease_button"
                if(action.action_name.equals("elevator_decrease_button")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        Robot.elevator.decreaseLevel();
                }
                //Event: "intake_hatch_toggle"
                if(action.action_name.equals("intake_hatch_toggle")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number)){
                        Robot.intake.toggleHatchIntake();
                        if(current_profile.has_rumble_feedback)
                            Rumble(current_profile.joysticks[action.controller_number], 0.5, 0.5);
                    }
                }
                //Event: "hatch_drop_toggle"
                if(action.action_name.equals("hatch_drop_toggle")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        Robot.intake.toggleHatchDrop();
                }
                //Event: "gear_change_toggle" 
                if(action.action_name.equals("gear_change_toggle")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number)){
                        Robot.driveTrain.toggleHighGear();
                        if(current_profile.has_rumble_feedback)
                            Rumble(current_profile.joysticks[action.controller_number], 0.5, 0.5);
                    }
                }
                //Event: "reset_button"
                if(action.action_name.equals("reset_button")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number)){
                        Robot.elevator.resetPosition();
                    }
                }
                //Event: "reverse_control_left"
                if(action.action_name.equals("reverse_control_left")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        current_profile.left_reverse *= -1;
                }
                //Event: "reverse_control_right"
                if(action.action_name.equals("reverse_control_right")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        current_profile.right_reverse *= -1;
                }
                //Event: "activate_vision"
                if(action.action_name.equals("activate_vision")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        Robot.driveTrain.setStatus(DriverStatus.Vision);
                }
                //Event: "cancel_vision"
                if(action.action_name.equals("cancel_vision")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number) && 
                            Robot.driveTrain.getStatus() == DriverStatus.Vision)
                        Robot.driveTrain.setStatus(DriverStatus.Control);
                }
                //Event:"lift_toggle"
                if(action.action_name.equals("lift_toggle")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        Robot.lift.toggleLift();
                }
                //Event:"forward_lock"
                if(action.action_name.equals("forward_lock")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonPressed(action.joystick_number))
                        Robot.driveTrain.lockForward();
                }
                //Event: "elevator_manual_mode"
                if(action.action_name.equals("elevator_manual_mode")){
                    if(current_profile.joysticks[action.controller_number].getRawButton(action.joystick_number)){
                        elevatorManualMode = true;
                    }
                }

                break;
                case ButtonHold:
                
                //Event: "intake_cargo_button"
                if(action.action_name.equals("intake_cargo_button")){
                    if(current_profile.joysticks[action.controller_number].getRawButton(action.joystick_number)){
                        Robot.intake.setCargoMotor(Robot.intake.cargoSpeed);
                        cargoMoving = true;
                    }
                }
                //Event: "outtake_cargo_button"
                if(action.action_name.equals("outtake_cargo_button")){
                    if(current_profile.joysticks[action.controller_number].getRawButton(action.joystick_number)){
                        Robot.intake.setCargoMotor(-Robot.intake.cargoSpeed);
                        cargoMoving = true;
                    }
                }
                //Event: "elevator_increase_button_open"
                if(action.action_name.equals("elevator_increase_button_open")){
                    if(current_profile.joysticks[action.controller_number].getRawButton(action.joystick_number)){
                        Robot.elevator.setPercent(1.0);
                        elevatorMove = true;
                    }
                }
                //Event: "elevator_decrease_button_open"
                if(action.action_name.equals("elevator_decrease_button_open")){
                    if(current_profile.joysticks[action.controller_number].getRawButton(action.joystick_number)){
                        Robot.elevator.setPercent(-0.75);
                        elevatorMove = true;
                    }
                }

                break;
                case ButtonRelease:
                //Event:"forward_lock"
                if(action.action_name.equals("forward_lock")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonReleased(action.joystick_number))
                        Robot.driveTrain.unlockForward();
                }
                //Event: "elevator_lock"
                if(action.action_name.equals("elevator_lock")){
                    if(current_profile.joysticks[action.controller_number].getRawButtonReleased(action.joystick_number)){
                        elevatorManualMode = false;
                        Robot.elevator.lockPos();
                    }
                }
                break;
                case POV:

                //Event: "elevetor_increase_POV"
                if(action.action_name.equals("elevetor_increase_POV")){
                    if(current_profile.joysticks[action.controller_number].getPOV() == action.joystick_number && !action.isPressed){
                        Robot.elevator.increaseLevel();
                        //Robot.elevator.setPercent(1.0);
                        action.isPressed = true;
                        elevatorMove = true;
                        if(current_profile.has_rumble_feedback)
                            Rumble(current_profile.joysticks[action.controller_number], 0.75, 0.5);
                    }else if(current_profile.joysticks[action.controller_number].getPOV() == -1 && action.isPressed){
                        action.isPressed = false;
                    }
                }
                //Event: "elevator_decrease_POV"
                if(action.action_name.equals("elevator_decrease_POV")){
                    if(current_profile.joysticks[action.controller_number].getPOV() == action.joystick_number && !action.isPressed){
                        Robot.elevator.decreaseLevel();
                        //Robot.elevator.setPercent(-0.75);
                        action.isPressed = true;
                        elevatorMove = true;
                        if(current_profile.has_rumble_feedback)
                            Rumble(current_profile.joysticks[action.controller_number], 0.75, 0.5);
                    }else if(current_profile.joysticks[action.controller_number].getPOV() == -1 && action.isPressed){
                        action.isPressed = false;
                    }
                }
                //Event: "elevator_loading_POV"
                if(action.action_name.equals("elevator_loading_POV")){
                    if(current_profile.joysticks[action.controller_number].getPOV() == action.joystick_number && !action.isPressed){
                        Robot.elevator.setLevel(5);
                        intakeManualMode = true;
                        action.isPressed = true;
                        elevatorMove = true;
                    }else if(current_profile.joysticks[action.controller_number].getPOV() == -1 && action.isPressed){
                        if(Robot.elevator.getLevel() == 5)
                            Robot.elevator.setLevel(0);
                        action.isPressed = false;
                        intakeManualMode = false;
                    }
                }

                break;
            }
        }
        if(!cargoMoving && !intakeManualMode)
            Robot.intake.setCargoMotor(0);
        //if(!elevatorMove)
            //Robot.elevator.setPercent(0);
    }

    private boolean isRumbleing = false;
    private double rumble_count = 0, rumble_length = 0;
    private Joystick rumble_controller;
    public void rumblePeriodic(){
        if(isRumbleing){
            if(rumble_count >= rumble_length){
                System.out.println("Rumble rumble");
                rumble_controller.setRumble(RumbleType.kLeftRumble, 0);
                rumble_controller.setRumble(RumbleType.kRightRumble, 0);
                rumble_count = 0;
                isRumbleing = false;
            }
            rumble_count++;
        }
    }

    public void setProfile(DriverProfile profile){
        current_profile = profile;
    }

    void CreateTroyProfile(){
        final int LEFT_CONTROLLER = 0;
        final int RIGHT_CONTROLLER = 1;

        DriverProfile troy = new DriverProfile("Troy", ControllerType.Attack3, ControlStyle.TankDrive, 2, false);
        JoystickAction movement = new JoystickAction("move_open", LEFT_CONTROLLER, 1, EventType.Axis, 0.2);
        JoystickAction movement_alt = new JoystickAction("move_open", RIGHT_CONTROLLER, 1, EventType.Axis, 0.2);
        movement.setSecondaryAction(movement_alt);
        troy.AddAction(movement);
        troy.AddAction(new JoystickAction("elevator_increase_button_open", LEFT_CONTROLLER, 3, EventType.ButtonHold));
        troy.AddAction(new JoystickAction("elevator_decrease_button_open", LEFT_CONTROLLER, 2, EventType.ButtonHold));
        troy.AddAction(new JoystickAction("intake_hatch_toggle", LEFT_CONTROLLER, 1, EventType.ButtonPress));
        troy.AddAction(new JoystickAction("hatch_drop_toggle", LEFT_CONTROLLER, 5, EventType.ButtonPress));
        troy.AddAction(new JoystickAction("gear_change_toggle", RIGHT_CONTROLLER, 5, EventType.ButtonPress));
        troy.AddAction(new JoystickAction("reset_button", LEFT_CONTROLLER, 6, EventType.ButtonPress));
        troy.AddAction(new JoystickAction("reverse_control_left", LEFT_CONTROLLER, 9, EventType.ButtonPress));
        troy.AddAction(new JoystickAction("reverse_control_right", RIGHT_CONTROLLER, 9, EventType.ButtonPress));
        troy.AddAction(new JoystickAction("lift_toggle", LEFT_CONTROLLER, 7, EventType.ButtonPress));

        troy.AddAction(new JoystickAction("intake_cargo_button", RIGHT_CONTROLLER, 1, EventType.ButtonHold));
        troy.AddAction(new JoystickAction("outtake_cargo_button", RIGHT_CONTROLLER, 3, EventType.ButtonHold));
        profiles.add(troy);
    }

    void CreateTroy310Profile(){
        final int F310 = 0;
        DriverProfile troy_f310 = new DriverProfile("Troy But Better", ControllerType.F310, ControlStyle.TankDrive, 1, false);
        
        //troy_f310.AddAction(moveClosed(F310));
        troy_f310.AddAction(moveSidways(F310));

        troy_f310.AddAction(new JoystickAction("intake_cargo_axis", F310, 2, EventType.Axis));
        troy_f310.AddAction(new JoystickAction("outtake_cargo_axis", F310, 3, EventType.Axis));

        troy_f310.AddAction(new JoystickAction("intake_hatch_toggle", F310, 1, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("hatch_drop_toggle", F310, 4, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("gear_change_toggle", F310, 8, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("reverse_control_left", F310, 9, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("reverse_control_right", F310, 10, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("lift_toggle", F310, 7, EventType.ButtonPress));
        //troy_f310.AddAction(new JoystickAction("forward_lock", F310, 3, EventType.ButtonPress));
        //troy_f310.AddAction(new JoystickAction("forward_lock", F310, 3, EventType.ButtonRelease));
        troy_f310.AddAction(new JoystickAction("activate_vision", F310, 3, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("elevator_manual_mode", F310, 2, EventType.ButtonPress));
        troy_f310.AddAction(new JoystickAction("elevator_lock", F310, 2, EventType.ButtonRelease));

        troy_f310.AddAction(new JoystickAction("elevetor_increase_POV", F310, 0, EventType.POV));
        troy_f310.AddAction(new JoystickAction("elevator_decrease_POV", F310, 180, EventType.POV));

        troy_f310.AddAction(new JoystickAction("elevator_loading_POV", F310, 90, EventType.POV));
        troy_f310.AddAction(new JoystickAction("elevator_loading_POV", F310, 270, EventType.POV));
        profiles.add(troy_f310);
    }

    JoystickAction moveClosed(int controller){
        JoystickAction action = new JoystickAction("move_closed", controller, 1, EventType.Axis);
        JoystickAction alt = new JoystickAction("move_closed", controller, 5, EventType.Axis);
        action.setSecondaryAction(alt);
        return action;
    }

    JoystickAction moveSidways(int controller){
        JoystickAction action = new JoystickAction("move_sideways_closed", controller, 1, EventType.Axis);
        JoystickAction alt = new JoystickAction("move_sideways_closed", controller, 4, EventType.Axis);
        action.setSecondaryAction(alt);
        return action;
    }

    public void Rumble(Joystick controller, double value, double length){
        System.out.println("Rumble at: " + value + ", for " + length);
        controller.setRumble(RumbleType.kLeftRumble, 1);
        controller.setRumble(RumbleType.kRightRumble, 1);
        rumble_controller = controller;
        rumble_length = length*50;//Converts seconds(s) to per 0.02 seconds(s)
        rumble_count = 0;
        isRumbleing = true;
    }
}