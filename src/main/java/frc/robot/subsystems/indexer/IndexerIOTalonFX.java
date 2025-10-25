package frc.robot.subsystems.indexer;

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

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX Indexer = new TalonFX(IndexerConstants.indexerCanId);
    private final StatusSignal<Angle> positionRot = Indexer.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSec = Indexer.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = Indexer.getMotorVoltage();
    private final StatusSignal<Current> currentAmps = Indexer.getSupplyCurrent();

    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public IndexerIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> Indexer.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        Indexer.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runPowerDutyCycleOut(double power) {
        Indexer.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }
}