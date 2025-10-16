package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
    public static final int elevatorMainCanId = 5;
    public static final int elevatorFollowerCanId = 15;
    public static final int currentLimit = 40;

    public static final double motorReduction = 175/18;
    public static final double drumDiameter = Units.inchesToMeters(1.751);
    public static final double rotationsToMeters = Math.PI * drumDiameter * 2;
    
    public static final double maxHeight = 1.25205255;

    public static final double coralScoringDownOffset = 0.08;

    public static TrapezoidProfile.Constraints profileConstraints =
        new TrapezoidProfile.Constraints(100.0, 26.0);

    public static final Gains gains =
        switch (Constants.getMode()) {
            case SIM -> new Gains(20.0, 0.0, 5.0, 0.0, 0.115, 0.0, 0.0);
            default -> new Gains(1,0,0,0,0, 0.0, 0.0);
        };

    // public static final motionMagicConstraints motionMagicConstraints =
    //     new motionMagicConstraints(500.0, 110.0, 4000.0);

    // public record motionMagicConstraints(double velocity, double acceleration, double jerk) {}

    public record Gains(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}