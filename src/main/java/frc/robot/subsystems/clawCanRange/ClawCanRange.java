package frc.robot.subsystems.clawCanRange;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.clawIntake.ClawIntakeConstants;

public class ClawCanRange extends SubsystemBase{
    
    public boolean isCoralIntaking = false;
    public boolean isBallIntaking = false;
    public boolean objectInClaw;

    private final CANrange RangeFinder;

    public ClawCanRange(){
        RangeFinder = new CANrange(ClawIntakeConstants.ClawIntakeCanId);
        var CanRangeConfig = new CANrangeConfiguration();
        CanRangeConfig.ProximityParams.withProximityThreshold(0.08);
        CanRangeConfig.ProximityParams.withMinSignalStrengthForValidMeasurement(0.02);
        CanRangeConfig.ProximityParams.withProximityHysteresis(0.039);
        tryUntilOk(5, () -> RangeFinder.getConfigurator().apply(CanRangeConfig, 0.25));
    }


    @Override
    public void periodic() {
        objectInClaw = RangeFinder.getIsDetected().getValue();
        Logger.recordOutput("ClawCanRange/ObjectDetected", objectInClaw);
        Logger.recordOutput("ClawCanRange/Connected", RangeFinder.isConnected());
    }
}
