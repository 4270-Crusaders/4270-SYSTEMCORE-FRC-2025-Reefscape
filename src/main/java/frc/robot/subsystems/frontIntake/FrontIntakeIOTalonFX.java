package frc.robot.subsystems.frontIntake;

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
import frc.robot.subsystems.indexer.Indexer;

import static frc.robot.util.PhoenixUtil.*;

public class FrontIntakeIOTalonFX implements FrontIntakeIO {
    private final TalonFX FrontIntake = new TalonFX(FrontIntakeConstants.frontIntakeCanId);
    private final StatusSignal<Angle> positionRot = FrontIntake.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSec = FrontIntake.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = FrontIntake.getMotorVoltage();
    private final StatusSignal<Current> currentAmps = FrontIntake.getSupplyCurrent();

    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public FrontIntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = FrontIntakeConstants.currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> FrontIntake.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    }

    @Override
    public void updateInputs(FrontIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        FrontIntake.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runPowerDutyCycleOut(double power) {
        FrontIntake.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }
}