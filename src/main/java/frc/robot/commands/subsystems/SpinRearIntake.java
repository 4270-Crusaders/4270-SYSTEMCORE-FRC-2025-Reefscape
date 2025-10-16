package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rearIntake.RearIntake;

public class SpinRearIntake extends Command {
    RearIntake s_Intake;
    double power;

    public SpinRearIntake(RearIntake subsys, double power) {
        s_Intake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_Intake.runPowerDutyCycleOut(power);
    }
}
