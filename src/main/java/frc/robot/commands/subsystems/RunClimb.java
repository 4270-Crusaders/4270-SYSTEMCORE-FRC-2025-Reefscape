package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbGoal;

public class RunClimb extends Command {
    Climb s_Climb;
    ClimbGoal pos;

    public RunClimb(Climb subsys, ClimbGoal pos) {
        s_Climb = subsys;
        this.pos = pos;

        addRequirements(subsys);
    }
    
    @Override
    public void execute() {
        s_Climb.getSetpointCommand(pos);
    }

    @Override
    public boolean isFinished() {
        return s_Climb.atGoal();
    }
}
