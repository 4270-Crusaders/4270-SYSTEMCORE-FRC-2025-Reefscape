package frc.robot.subsystems.rearIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.util.PhoenixUtil.*;

public class RearIntakeIOTalonFX implements RearIntakeIO {
    private final TalonFX RearIntake = new TalonFX(RearIntakeConstants.rearIntakeCanId);
    private final StatusSignal<Angle> positionRot = RearIntake.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSec = RearIntake.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = RearIntake.getMotorVoltage();
    private final StatusSignal<Current> currentAmps = RearIntake.getSupplyCurrent();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public RearIntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = RearIntakeConstants.currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> RearIntake.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    }

    @Override
    public void updateInputs(RearIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        RearIntake.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runPowerDutyCycleOut(double power) {
        RearIntake.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }

}