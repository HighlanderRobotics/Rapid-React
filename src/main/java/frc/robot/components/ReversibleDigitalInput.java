package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;
import io.github.oblarg.oblog.Loggable;

/**A wrapper over DigitalInput that lets the returned boolean be reversed if the sensor was wired different then expected. */
public class ReversibleDigitalInput implements Loggable {
    
    final boolean reversed;
    final DigitalInput input;
    public ReversibleDigitalInput(int channel, boolean reversed) {
        input = new DigitalInput(channel);
        this.reversed = reversed;
    }

    /**Returns the state of the input, reversed if the DI is set to be reversed. */
    public boolean get(){
        return input.get() != reversed;
    }

}