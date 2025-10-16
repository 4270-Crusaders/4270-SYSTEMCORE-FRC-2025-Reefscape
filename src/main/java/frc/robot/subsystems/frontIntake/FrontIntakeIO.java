package frc.robot.subsystems.frontIntake;

import org.littletonrobotics.junction.AutoLog;

public interface FrontIntakeIO {
    @AutoLog
    public static class FrontIntakeIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(FrontIntakeIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    public default void runPowerDutyCycleOut(double power) {}
}