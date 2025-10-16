package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean MotorConnected = true;
        public double positionDegrees = 0.0;
        public double positionRotation = 0.0;
        public double velocityRotationsPerSecond = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double setpointDegrees = 0.0;
        
        public boolean atGoal = false;
    }

    default boolean atGoal() {
        return false;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void stop() {}

    default void runVolts(double volts) {}

    default void runSetpoint(double setpointDegrees, double feedforward) {}

    default void runSetpointMotionMagic(double setpointDegrees, double feedforward, double maxVelocity) {}

    default void runCurrent(double amps) {}

    default void setPID(double p, double i, double d, double v, double s, double a, double g) {}
}
