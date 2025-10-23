package frc.robot.commands.alignment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagObj;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.FieldConstants.CoralLevels;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ChassisSpeedConvert;
import frc.robot.util.LoggedTunableNumber;

public class AlignToPole extends Command{
    private Double distanceBackFromTagOffset;
    private final Double distanceSideFromTagOffsetLeft=0.185;
    private final Double distanceSideFromTagOffsetRight=-0.18;

    LoggedTunableNumber L1OffsetDistanceTunableNumber = new LoggedTunableNumber("Align/Offsets/L1OffsetDistance", -0.45);
    LoggedTunableNumber L2OffsetDistanceTunableNumber = new LoggedTunableNumber("Align/Offsets/L2OffsetDistance", -0.5);
    LoggedTunableNumber L3OffsetDistanceTunableNumber = new LoggedTunableNumber("Align/Offsets/L3OffsetDistance", -0.45);
    LoggedTunableNumber L4OffsetDistanceTunableNumber = new LoggedTunableNumber("Align/Offsets/L4OffsetDistance", -0.45);

    double L1OffsetDistance;
    double L2OffsetDistance;
    double L3OffsetDistance;
    double L4OffsetDistance;

    Drive drive;

    double velocityX;
    double velocityY;
    double velocityRot;

    PIDController HeadingController;
    LoggedTunableNumber kPLinTunableNumber = new LoggedTunableNumber("Align/KpLin", 3.2);
    LoggedTunableNumber kPRotTunableNumber = new LoggedTunableNumber("Align/KpRot", 4.2);
    
    double kP_lin;
    double kP_rot;
    double sideOffset;

    public Pose2d goalPose;
    public AprilTagObj TagObj;
    public int TagId;

    CoralLevels currentCoralLevel;

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
        L1OffsetDistance = L1OffsetDistanceTunableNumber.get();
        L2OffsetDistance = L2OffsetDistanceTunableNumber.get();
        L3OffsetDistance = L3OffsetDistanceTunableNumber.get();
        L4OffsetDistance = L4OffsetDistanceTunableNumber.get();
    }

    @Override
    public void execute() {
        currentCoralLevel = RobotContainer.elevator.getCurrentLevel();
        LoggedTunableNumber.ifChanged(
            hashCode(),
            ()->changeVal(),
            kPLinTunableNumber,
            kPRotTunableNumber,
            L1OffsetDistanceTunableNumber,
            L2OffsetDistanceTunableNumber,
            L3OffsetDistanceTunableNumber,
            L4OffsetDistanceTunableNumber);

        switch (currentCoralLevel) {
            case L1:
                distanceBackFromTagOffset = L1OffsetDistance;
                break;
            case L2:
                distanceBackFromTagOffset = L2OffsetDistance;
                break;
            case L3:
                distanceBackFromTagOffset = L3OffsetDistance;
                break;
            case L4:
                distanceBackFromTagOffset = L4OffsetDistance;
                break;
            default:
                distanceBackFromTagOffset = L4OffsetDistance;
                break;
        }

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

        drive.runVelocity(ChassisSpeedConvert.fromFieldRelativeSpeeds(new ChassisSpeeds(velocityX, velocityY, OmegaVelocity),currentRotation));
    }
}