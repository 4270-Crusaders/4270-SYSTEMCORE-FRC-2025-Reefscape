package frc.robot.commands.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;

public class RunElevator extends Command {
    Elevator s_Elevator;
    ElevatorGoal rPos;

    public RunElevator(Elevator subsys, ElevatorGoal rPos) {
        s_Elevator = subsys;
        this.rPos = rPos;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Elevator.getSetpointCommand(rPos);
    }

    @Override
    public boolean isFinished() {
        return s_Elevator.atGoal();
    }
}
