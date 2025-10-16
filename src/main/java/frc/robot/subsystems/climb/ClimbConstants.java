package frc.robot.subsystems.climb;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class ClimbConstants {
    public static final int climbCanId = 4;
    public static final int currentLimit = 50;

    public static final double motorReduction = 1;
    public static final double armLength = 20;

    public static TrapezoidProfile.Constraints profileConstraints =
        new TrapezoidProfile.Constraints(100.0, 26.0);

    public static final Gains gains =
        switch (Constants.getMode()) {
            case SIM -> new Gains(20.0, 0.0, 5.0, 0.0, 0.115, 0.0, 0.0);
            default -> new Gains(1,0,0,0,0, 0.0, 0.0);
        };

    public record Gains(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}