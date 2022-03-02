package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LimitSwitch implements Loggable {
    
    final boolean reversed; 
    final DigitalInput input;
    public LimitSwitch(int channel, boolean reversed) {
        input = new DigitalInput(channel);
        this.reversed= reversed;
    }

    public boolean get(){
        return input.get() != reversed;
    }
}