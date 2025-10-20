package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.leds.LedConstants.LEDStates;

public class SetLEDs extends Command {
    Led s_LEDs;
    LEDStates state;
    boolean finished;

    public SetLEDs(Led subsys, LEDStates state) {
        finished = false;
        s_LEDs = subsys;
        this.state = state;
        addRequirements(subsys);
    }
    
    @Override
    public void initialize() {
        finished=false;
    }

    @Override
    public void execute() {
        finished = false;
        s_LEDs.setLEDS(state);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
