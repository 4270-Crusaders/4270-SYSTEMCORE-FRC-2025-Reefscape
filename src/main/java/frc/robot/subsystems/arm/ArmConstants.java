package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmConstants {
    public static final int ARM_CAN_ID = 34;
    public static final int ARM_SENSOR_CAB_ID = 59;
    public static final int currentLimit = 60;

    public static final double armLength = 20;

    public static final Gains gains =
        switch (Constants.getMode()) {
            case SIM -> new Gains(20.0, 0.0, 5.0, 0.0, 0.115, 0.0, 0.0);
            default -> new Gains(25,0,0,1.0,0.25, 0.05, 0.23);
        };

    public record Gains(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}