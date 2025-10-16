package frc.robot.subsystems.intakeWrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class IntakeWristConstants {
    public static final int intakeWristCanId = 14;
    public static final int currentLimit = 40;

    public static final double motorReduction = (75*9)/11;
    public static final double armLength = 20;

    public static TrapezoidProfile.Constraints profileConstraints =
        new TrapezoidProfile.Constraints(0.0, 0.0);

    public static final Gains gains =
        switch (Constants.getMode()) {
            case SIM -> new Gains(20.0, 0.0, 5.0, 0.0, 0.115, 0.0, 0.0);
            default -> new Gains(0.85,0,0,0.0,0.0, 0.0, 0.0);
        };

    public record Gains(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}