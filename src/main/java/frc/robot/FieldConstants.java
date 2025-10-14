package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    // Pole Side
    public static enum PoleSide {
        Left,
        Right,
        Middle
    }

    // coral levels
    public enum CoralLevels {
        L1,
        L2,
        L3,
        L4,
        L4Auto
    }
    public static final AprilTagObj[] CORAL_APRIL_TAG_OBJS = {
        //blue
        new AprilTagObj(17, new Pose2d(4.07, 3.31, Rotation2d.fromDegrees(240))),
        new AprilTagObj(18, new Pose2d(3.66, 4.03, Rotation2d.fromDegrees(180))),
        new AprilTagObj(19, new Pose2d(4.07, 4.75, Rotation2d.fromDegrees(120))),
        new AprilTagObj(20, new Pose2d(4.90, 4.75, Rotation2d.fromDegrees(60))),
        new AprilTagObj(21, new Pose2d(5.32, 4.03, Rotation2d.fromDegrees(0))),
        new AprilTagObj(22, new Pose2d(4.90, 3.31, Rotation2d.fromDegrees(300))),

        new AprilTagObj(6, new Pose2d(13.47, 3.31, Rotation2d.fromDegrees(300))),
        new AprilTagObj(7, new Pose2d(13.89, 4.03, Rotation2d.fromDegrees(0))),
        new AprilTagObj(8, new Pose2d(13.47, 4.75, Rotation2d.fromDegrees(60))),
        new AprilTagObj(9, new Pose2d(12.64, 4.75, Rotation2d.fromDegrees(120))),
        new AprilTagObj(10, new Pose2d(12.23, 4.03, Rotation2d.fromDegrees(180))),
        new AprilTagObj(11, new Pose2d(12.64, 3.31, Rotation2d.fromDegrees(240)))
    };
    
    public static final Pose2d[] allApriltagPoses(){
        Pose2d[] poses = new Pose2d[CORAL_APRIL_TAG_OBJS.length];
        for(int i = 0; i < CORAL_APRIL_TAG_OBJS.length; i++){
            poses[i] = CORAL_APRIL_TAG_OBJS[i].getTagPose();
        }
        return poses;
    }

    public static AprilTagObj getTagObjByPose(Pose2d pose){
        double poseX = pose.getX();
        double poseY = pose.getY();

        for(AprilTagObj tag: CORAL_APRIL_TAG_OBJS){
            if(poseX == tag.getTagPose().getX() && poseY == tag.getTagPose().getY()){
                return tag;
            }
        }

        return new AprilTagObj(-1, pose);
    }
}
