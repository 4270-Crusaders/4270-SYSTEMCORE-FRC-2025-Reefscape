package frc.robot.subsystems.rearIntake;

import org.littletonrobotics.junction.AutoLog;

public interface RearIntakeIO {
    @AutoLog
    public static class RearIntakeIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(RearIntakeIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}
    
    /** Run open loop at the specified power. */
    public default void runPowerDutyCycleOut(double power) {}
}