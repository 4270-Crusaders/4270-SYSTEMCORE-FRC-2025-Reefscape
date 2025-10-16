package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmGoal;

public class RunArm extends Command {
    Arm s_Arm;
    ArmGoal pos;
    double maxVelo;

    public RunArm(Arm subsys, ArmGoal pos, double maxVelo) {
        s_Arm = subsys;
        this.pos = pos;
        this.maxVelo = maxVelo;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Arm.getSetpointCommand(pos, maxVelo);
    }

    @Override
    public boolean isFinished() {
        return s_Arm.atGoal();
    }
}
