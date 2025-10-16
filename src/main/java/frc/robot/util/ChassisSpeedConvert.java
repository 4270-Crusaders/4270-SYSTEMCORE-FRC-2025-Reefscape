package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class ChassisSpeedConvert {
    /**
     * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond The component of speed in the x direction relative to the field.
     *     Positive x is away from your alliance wall.
     * @param vyMetersPerSecond The component of speed in the y direction relative to the field.
     *     Positive y is to your left when standing behind the alliance wall.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        Rotation2d robotAngle) {
        // CW rotation into chassis frame
        var rotated =
            new Translation2d(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
     * object.
     *
     * @param vx The component of speed in the x direction relative to the field. Positive x is away
     *     from your alliance wall.
     * @param vy The component of speed in the y direction relative to the field. Positive y is to
     *     your left when standing behind the alliance wall.
     * @param omega The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(
        LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2d robotAngle) {
        return fromFieldRelativeSpeeds(
            vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), robotAngle);
    }

    /**
     * Converts a user provided field-relative ChassisSpeeds object into a robot-relative
     * ChassisSpeeds object.
     *
     * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds in the field frame
     *     of reference. Positive x is away from your alliance wall. Positive y is to your left when
     *     standing behind the alliance wall.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(
        ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
        return fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vx,
            fieldRelativeSpeeds.vy,
            fieldRelativeSpeeds.omega,
            robotAngle);
    }

    /**
     * Converts a user provided robot-relative set of speeds into a field-relative ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond The component of speed in the x direction relative to the robot.
     *     Positive x is towards the robot's front.
     * @param vyMetersPerSecond The component of speed in the y direction relative to the robot.
     *     Positive y is towards the robot's left.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of reference.
     */
    public static ChassisSpeeds fromRobotRelativeSpeeds(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        Rotation2d robotAngle) {
        // CCW rotation out of chassis frame
        var rotated = new Translation2d(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle);
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
    }

    /**
     * Converts a user provided robot-relative set of speeds into a field-relative ChassisSpeeds
     * object.
     *
     * @param vx The component of speed in the x direction relative to the robot. Positive x is
     *     towards the robot's front.
     * @param vy The component of speed in the y direction relative to the robot. Positive y is
     *     towards the robot's left.
     * @param omega The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of reference.
     */
    public static ChassisSpeeds fromRobotRelativeSpeeds(
        LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2d robotAngle) {
        return fromRobotRelativeSpeeds(
            vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), robotAngle);
    }

    /**
     * Converts a user provided robot-relative ChassisSpeeds object into a field-relative
     * ChassisSpeeds object.
     *
     * @param robotRelativeSpeeds The ChassisSpeeds object representing the speeds in the robot frame
     *     of reference. Positive x is towards the robot's front. Positive y is towards the robot's
     *     left.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of reference.
     */
    public static ChassisSpeeds fromRobotRelativeSpeeds(
        ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
        return fromRobotRelativeSpeeds(
            robotRelativeSpeeds.vx,
            robotRelativeSpeeds.vy,
            robotRelativeSpeeds.omega,
            robotAngle);
    }
}
