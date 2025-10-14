package frc.robot.commands.alignment;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

public class PathPlannerAlignToPole extends Command{
    Drive drive;
    Double distanceBackFromTagOffset=-0.45;
    Double distanceSideFromTagOffset=0.18;
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRate = RotationsPerSecond.of(10).in(RadiansPerSecond);

    Pose2d targetPose;
    // Create the constraints to use while pathfinding
    PathConstraints constraints;

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand;
    PoleSide side;

    public PathPlannerAlignToPole(Drive drive, PoleSide side) {
        this.drive = drive;
        this.side = side;

        // Create the constraints to use while pathfinding
        constraints = new PathConstraints(
            MaxSpeed,20,
                MaxAngularRate, Units.rotationsToRadians(5));
    }
    
    @Override
    public void execute() {
        Pose2d tagPose = drive.findClosestApriltag();
        
        Rotation2d tagRotation = tagPose.getRotation();

        // Determine side offset relative to tag
        double sideOffset = (side == PoleSide.Left)
            ? distanceSideFromTagOffset
            : -distanceSideFromTagOffset;

        // Compute goal pose in field space
        double goalX = tagPose.getX()
            - distanceBackFromTagOffset * Math.cos(tagRotation.getRadians())
            + sideOffset * Math.sin(tagRotation.getRadians());

        double goalY = tagPose.getY()
            - distanceBackFromTagOffset * Math.sin(tagRotation.getRadians())
            - sideOffset * Math.cos(tagRotation.getRadians());

        // Keep the tag's rotation or reverse it (180°) if you want to face the tag
        Rotation2d goalRotation = tagRotation.plus(Rotation2d.fromDegrees(180));

        targetPose = new Pose2d(goalX, goalY, goalRotation);
        Logger.recordOutput("AlignmentThings/targetPose", targetPose);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity in meters/sec
        );

        pathfindingCommand.schedule();
    }
}
