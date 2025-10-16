package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.clawIntake.ClawIntake;

public class SpinClawIntake extends Command {
    ClawIntake s_ClawIntake;
    double power;
    
    public SpinClawIntake(ClawIntake subsys, double power) {
        s_ClawIntake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_ClawIntake.runPowerDutyCycleOut(power);
    }
}
