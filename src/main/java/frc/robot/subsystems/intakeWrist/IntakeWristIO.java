package frc.robot.subsystems.intakeWrist;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWristIO {
    @AutoLog
    class IntakeWristIOInputs {
        public boolean MotorConnected = true;
        public double positionRotations = 0.0;
        public double positionDegrees = 0.0;
        public double velocityRotationsPerSecond = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double setpointRotations = 0.0;
        
        public boolean atGoal = false;
    }

    default boolean atGoal() {
        return false;
    }

    default void updateInputs(IntakeWristIOInputs inputs) {}

    default void stop() {}

    default void runVolts(double volts) {}

    default void runSetpoint(double setpointDegrees, double feedforward) {}

    default void runSetpointMotionMagic(double setpointDegrees, double feedforward) {}

    default void runCurrent(double amps) {}

    default void setPID(double p, double i, double d, double v, double s, double a, double g) {}
}
