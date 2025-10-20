package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.clawIntake.ClawIntake;

public class SpinClawIntake extends Command {
    ClawIntake s_ClawIntake;
    double power;
    boolean finished;

    public SpinClawIntake(ClawIntake subsys, double power) {
        finished = false;
        s_ClawIntake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        finished = false;
        s_ClawIntake.runPowerDutyCycleOut(power);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
