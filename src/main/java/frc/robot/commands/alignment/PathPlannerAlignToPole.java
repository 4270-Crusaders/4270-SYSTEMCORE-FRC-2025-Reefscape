package frc.robot.commands.alignment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.commands.pathOnFly.PathFindToPoseCommand;
import frc.robot.subsystems.drive.Drive;

public class PathPlannerAlignToPole extends Command{
    Drive drive;
    Double distanceBackFromTagOffset=-0.45;
    Double distanceSideFromTagOffset=0.18;

    Pose2d targetPose;
    // Create the constraints to use while pathfinding

    PathFindToPoseCommand pathFindToPoseCommand;
    PoleSide side;

    public PathPlannerAlignToPole(Drive drive, PoleSide side) {
        this.drive = drive;
        this.side = side;

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

        new PathFindToPoseCommand(drive, targetPose, 0);
    }

    @Override
    public boolean isFinished() {
        return pathFindToPoseCommand.isFinished();
    }
}
