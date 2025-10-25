package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class SpinIndexer extends Command {
    Indexer s_Indexer;
    double power;

    public SpinIndexer(Indexer subsys, double power) {
        s_Indexer = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        // s_Indexer.runVoltage(power);
        s_Indexer.runPowerDutyCycleOut(power);
    }
}
