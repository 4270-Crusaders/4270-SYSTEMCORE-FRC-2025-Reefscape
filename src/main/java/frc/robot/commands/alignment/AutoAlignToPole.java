package frc.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagObj;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CoralLevels;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.EqualsUtil;


public class AutoAlignToPole extends Command{
    double rotationThreshold = 2; // degrees
    double distanceThreshold = 0.5; // meters

    PoleSide poleSide;
    Drive drive;
    Boolean isFinishedBool = false;
    AprilTagObj TagObj;
    AlignToPole aligningCommand;
    double distanceBetweenGoal;

    CoralLevels CoralLevel;
    public AutoAlignToPole(PoleSide poleside, Drive drive, CoralLevels CoralLevel){
        this.CoralLevel = CoralLevel;
        this.poleSide = poleside;
        this.drive = drive;
    }

    @Override
    public void execute() {
        isFinishedBool = false;
        while (!isFinishedBool){
            aligningCommand = new AlignToPole(poleSide, drive);
            
            Pose2d goalPose = aligningCommand.goalPose;

            distanceBetweenGoal = drive.getPose().getTranslation().getDistance(goalPose.getTranslation());

            TagObj = FieldConstants.getTagObjByPose(drive.findClosestApriltag());
            double goalRot = goalPose.getRotation().getDegrees();
            double robotRot = drive.getPose().getRotation().getDegrees();
            
            if (EqualsUtil.epsilonEquals(goalRot, robotRot, rotationThreshold)&& (distanceBetweenGoal < distanceThreshold)){
                isFinishedBool = true;
            }
            else {
                isFinishedBool = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinishedBool;
    }
}
