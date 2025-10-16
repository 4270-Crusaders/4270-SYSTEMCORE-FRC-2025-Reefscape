package frc.robot.subsystems.clawIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.util.PhoenixUtil.*;

public class ClawIntakeIOTalonFX implements ClawIntakeIO {
    private final TalonFX ClawIntake = new TalonFX(ClawIntakeConstants.ClawIntakeCanId);
    private final StatusSignal<Angle> positionRot = ClawIntake.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSec = ClawIntake.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = ClawIntake.getMotorVoltage();
    private final StatusSignal<Current> currentAmps = ClawIntake.getSupplyCurrent();
    
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public ClawIntakeIOTalonFX() {
    var MotorConfig = new TalonFXConfiguration();
    MotorConfig.CurrentLimits.SupplyCurrentLimit = ClawIntakeConstants.currentLimit;
    MotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    MotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    MotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    tryUntilOk(5, () -> ClawIntake.getConfigurator().apply(MotorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    }

    @Override
    public void updateInputs(ClawIntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

        inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        ClawIntake.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runPowerDutyCycleOut(double power) {
        ClawIntake.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }
}