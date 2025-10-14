package frc.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagObj;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.EqualsUtil;


public class AutoAlignToPole extends Command{
    PoleSide poleSide;
    Drive drive;
    Boolean isFinishedBool = false;
    AprilTagObj TagObj;
    AlignToPole aligningCommand;
    public AutoAlignToPole(PoleSide poleside, Drive drive){
        this.poleSide = poleside;
        this.drive = drive;
    }

    @Override
    public void execute() {
        while (!isFinishedBool){
            aligningCommand = new AlignToPole(poleSide, drive);
            
            Pose2d goalPose = aligningCommand.goalPose;
            TagObj = FieldConstants.getTagObjByPose(drive.findClosestApriltag());
            double goalX = goalPose.getX();
            double goalY = goalPose.getY();
            double goalRot = goalPose.getRotation().getDegrees();

            double robotX = drive.getPose().getTranslation().getX();
            double robotY = drive.getPose().getTranslation().getY();
            double robotRot = drive.getPose().getRotation().getDegrees();
            
            if (EqualsUtil.epsilonEquals(goalRot, robotRot, 2)&&
            EqualsUtil.epsilonEquals(goalX, robotX, 2)&&
            EqualsUtil.epsilonEquals(goalY, robotY, 2)){
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
