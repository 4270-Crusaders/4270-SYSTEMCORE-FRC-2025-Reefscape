package frc.robot.commands.pathOnFly;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

public class PathFindToPoseCommand extends Command{
    Drive drive;

    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRate = RotationsPerSecond.of(10).in(RadiansPerSecond);

    Pose2d targetPose;
    double goalEndVelocity;
    // Create the constraints to use while pathfinding
    PathConstraints constraints;

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand;

    public PathFindToPoseCommand(Drive drive, Pose2d targetPose, double goalEndVelocity) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.goalEndVelocity = goalEndVelocity;

        // Create the constraints to use while pathfinding
        constraints = new PathConstraints(
            MaxSpeed,20,
                MaxAngularRate, Units.rotationsToRadians(5));
    }
    
    @Override
    public void execute() {
        Logger.recordOutput("PathOnFly/LatestTargetPose", targetPose);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            goalEndVelocity // Goal end velocity in meters/sec
        );

        pathfindingCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return pathfindingCommand.isFinished();
    }
}
