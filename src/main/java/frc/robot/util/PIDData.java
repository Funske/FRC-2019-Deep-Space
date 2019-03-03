package frc.robot.util;

public class PIDData{
    
    private double pGain = 0, iGain = 0, dGain = 0, fGain;
    private int max_velocity = 0, max_acceleration = 0, max_error = 0;
    private int slot_index = 0, timeout = 10;

    public PIDData(double p, double i, double d, double f, int slot_index, int timeout){
        pGain = p;
        iGain = i;
        dGain = d;
        fGain = f;
        this.slot_index = slot_index;
        this.timeout = timeout;
    }

    public PIDData(double p, double i, double d){
        pGain = p;
        iGain = i;
        dGain = d;
    }
    
    public PIDData(double p, double i, double d, double f){
        pGain = p;
        iGain = i;
        dGain = d;
        fGain = f;
    } 

    public void setMaxVelocity(int velocity){
        max_velocity = velocity;
    }

    public void setMaxAcceleration(int acceleration){
        max_acceleration = acceleration;
    }

    public void setMaxError(int error){
        max_error = error;
    }
    
    public double getP(){
        return pGain;
    }

    public double getI(){
        return iGain;
    }

    public double getD(){
        return dGain;
    }

    public double getF(){
        return fGain;
    }

    public int getSlotIndex(){
        return slot_index;
    }

    public int getTimeout(){
        return timeout;
    }

    public int getMaxVelocity(){
        return max_velocity;
    }

    public int getMaxAcceleration(){
        return max_acceleration;
    }

    public int getMaxError(){
        return max_error;
    }
}