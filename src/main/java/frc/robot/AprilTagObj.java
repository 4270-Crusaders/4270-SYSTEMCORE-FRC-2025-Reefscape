package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagObj{
    private final int ID;
    private final Pose2d TagPose;

    public AprilTagObj(int ID, Pose2d TagPose){
        this.ID = ID;
        this.TagPose = TagPose;
    }

    public int getID(){
        return this.ID;
    }

    public Pose2d getTagPose(){
        return this.TagPose;
    }
}
