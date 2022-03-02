package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch extends DigitalInput{

    boolean reversed;
    public LimitSwitch(int channel, boolean reversed) {
        super(channel);
        reversed = this.reversed;
    }

    public boolean get(){
        return super.get() != reversed;
    }
}