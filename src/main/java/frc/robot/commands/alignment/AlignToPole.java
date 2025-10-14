package frc.robot.commands.alignment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagObj;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class AlignToPole extends Command{
    private final Double distanceBackFromTagOffset=-0.45;
    private final Double distanceSideFromTagOffsetLeft=0.185;
    private final Double distanceSideFromTagOffsetRight=-0.18;

    
    Drive drive;

    double velocityX;
    double velocityY;
    double velocityRot;

    PIDController HeadingController;
    LoggedTunableNumber kPLinTunableNumber = new LoggedTunableNumber("Align/KpLin", 3.67);
    LoggedTunableNumber kPRotTunableNumber = new LoggedTunableNumber("Align/KpRot", 4.2);
    
    double kP_lin;
    double kP_rot;
    double sideOffset;

    public Pose2d goalPose;
    public AprilTagObj TagObj;
    public int TagId;

    public AlignToPole (PoleSide side, Drive drive) {
        this.drive = drive;
        PoleSide currentPoleSide = side;
        // Making PID stuff
        HeadingController = new PIDController(kP_rot, 0.0, 0.0);
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Determine side offset relative to pole side
        if(currentPoleSide == PoleSide.Left){
            sideOffset = distanceSideFromTagOffsetLeft;
        }
        else if (currentPoleSide == PoleSide.Right)
            sideOffset = distanceSideFromTagOffsetRight;
        else{ // Middle
            sideOffset = 0;
        }
    }
    private void changeVal(){
        kP_lin = kPLinTunableNumber.get();
        kP_rot = kPRotTunableNumber.get();
    }
    @Override
    public void execute() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            ()->changeVal(),
            kPLinTunableNumber,
            kPRotTunableNumber);

        HeadingController.setP(kP_rot);

        TagObj = FieldConstants.getTagObjByPose(drive.findClosestApriltag());
        TagId = TagObj.getID();

        Logger.recordOutput("Alignment/FocusedTagID", TagId);
        Logger.recordOutput("Alignment/GoalPose", goalPose);

        Pose2d closestTagPose2d = TagObj.getTagPose();
        Rotation2d closestTagRotation = closestTagPose2d.getRotation();

        // Compute goal pose in field space
        double goalX = closestTagPose2d.getX()
            - distanceBackFromTagOffset * Math.cos(closestTagRotation.getRadians())
            + sideOffset * Math.sin(closestTagRotation.getRadians());

        double goalY = closestTagPose2d.getY()
            - distanceBackFromTagOffset * Math.sin(closestTagRotation.getRadians())
            - sideOffset * Math.cos(closestTagRotation.getRadians());

        // Keep the tag's rotation or reverse it (180°) if you want to face the tag
        Rotation2d goalRotation = closestTagRotation.plus(Rotation2d.fromDegrees(180));

        goalPose = new Pose2d(goalX, goalY, goalRotation);

        // Current robot pose from odometry
        Pose2d currentPose2d = drive.getPose();
        Rotation2d currentRotation = currentPose2d.getRotation();

        double OmegaVelocity = HeadingController.calculate(currentRotation.getRadians(), goalRotation.getRadians());

        // Translation Error Calculation
        double errorX = goalPose.getX() - currentPose2d.getX();
        double errorY = goalPose.getY() - currentPose2d.getY();

        velocityX = kP_lin * errorX;
        velocityY = kP_lin * errorY;

        drive.runVelocity(new ChassisSpeeds(velocityX, velocityY, OmegaVelocity).toFieldRelative(currentRotation));
    }
}

