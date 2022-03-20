package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config.Relay;

public class ReversibleDigitalInput implements Loggable {
    
    final boolean reversed; 
    final DigitalInput input;
    public ReversibleDigitalInput(int channel, boolean reversed) {
        input = new DigitalInput(channel);
        this.reversed = reversed;
    }

    public boolean get(){
        return input.get() != reversed;
    }

}