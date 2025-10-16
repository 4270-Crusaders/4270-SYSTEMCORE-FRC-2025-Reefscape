package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.leds.LedConstants.LEDStates;

public class SetLEDs extends Command {
    Led s_LEDs;
    LEDStates state;

    public SetLEDs(Led subsys, LEDStates state) {
        s_LEDs = subsys;
        this.state = state;

        addRequirements(subsys);
    }
    
    @Override
    public void initialize() {
        s_LEDs.setLEDS(state);
    }
}
