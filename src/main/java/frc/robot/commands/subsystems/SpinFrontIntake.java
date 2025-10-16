package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.frontIntake.FrontIntake;

public class SpinFrontIntake extends Command {
    FrontIntake s_Intake;
    double power;

    public SpinFrontIntake(FrontIntake subsys, double power) {
        s_Intake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_Intake.runPowerDutyCycleOut(power);
    }

    
}
