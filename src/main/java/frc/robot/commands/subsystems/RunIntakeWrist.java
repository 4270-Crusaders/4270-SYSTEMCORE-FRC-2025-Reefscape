package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intakeWrist.IntakeWrist.IntakeWristGoal;

public class RunIntakeWrist extends Command {
    IntakeWrist s_Wrist;
    IntakeWristGoal lPos;

    public RunIntakeWrist(IntakeWrist subsys, IntakeWristGoal lPos) {
        s_Wrist = subsys;
        this.lPos = lPos;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Wrist.getSetpointCommand(lPos);
    }

    @Override
    public boolean isFinished() {
        return s_Wrist.atGoal();
    }
}
