package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.climbCanId;
import static frc.robot.subsystems.climb.ClimbConstants.motorReduction;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.EqualsUtil;
import frc.robot.util.PhoenixUtil;

public class ClimbIOTalonFX implements ClimbIO{
    // Hardware
    private final TalonFX climbMotor;

    // Status Signals
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final StatusSignal<Double> setpointRotations;

    private double goalPositionRotations;

    // Control
    private final NeutralOut neutralOut = new NeutralOut();
    private final PositionVoltage positionControl = new PositionVoltage(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0.0);

    private final MotionMagicExpoVoltage motionMagicControl =
        new MotionMagicExpoVoltage(0.0);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    public ClimbIOTalonFX() {
        climbMotor = new TalonFX(climbCanId);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.MotionMagic.withMotionMagicCruiseVelocity(250);
        config.MotionMagic.withMotionMagicExpo_kV(0.002);
        config.MotionMagic.withMotionMagicExpo_kA(0.0005);

        climbMotor.getClosedLoopReference();

        PhoenixUtil.tryUntilOk(5, () -> climbMotor.getConfigurator().apply(config));

        positionRotations = climbMotor.getPosition();
        velocityRps = climbMotor.getVelocity();
        appliedVoltage = climbMotor.getMotorVoltage();
        supplyCurrent = climbMotor.getSupplyCurrent();
        torqueCurrent = climbMotor.getTorqueCurrent();
        tempCelsius = climbMotor.getDeviceTemp();
        setpointRotations = climbMotor.getClosedLoopReference();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            positionRotations,
            velocityRps,
            appliedVoltage,
            supplyCurrent,
            torqueCurrent,
            tempCelsius);

        goalPositionRotations = 0.0;
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.MotorConnected =
            BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                setpointRotations)
            .isOK();

        inputs.positionRotations = positionRotations.getValueAsDouble();
        inputs.velocityRotationsPerSecond = velocityRps.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.setpointRotations = setpointRotations.getValueAsDouble();
    }

    @Override
    public void stop() {
        climbMotor.setControl(neutralOut);
    }

    @Override
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
            climbMotor.getPosition().getValueAsDouble(), goalPositionRotations, 1.0);
    }

    @Override
    public void runVolts(double volts) {
        climbMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void runCurrent(double amps) {
        climbMotor.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
    }

    @Override
    public void setPID(double p, double i, double d, double v, double s, double a, double g) {
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        config.Slot0.kS = s;
        config.Slot0.kV = v;
        config.Slot0.kA = a;
        config.Slot0.kG = g;

        PhoenixUtil.tryUntilOk(5, () -> climbMotor.getConfigurator().apply(config));
    }


    @Override
    public void runSetpoint(double setpointRotations, double feedforward) {
        double setpoint = setpointRotations * motorReduction;
        Logger.recordOutput("Climb/SetpointDegrees", setpoint);

        climbMotor.setControl( // kG 0.5 kV 1000.
            positionControl.withPosition(setpoint).withEnableFOC(true).withFeedForward(feedforward));
    }

    @Override
    public void runSetpointMotionMagic(double setpointRotations, double feedforward) {
        goalPositionRotations = setpointRotations * motorReduction;
        Logger.recordOutput("Climb/SetpointDegrees", goalPositionRotations);
        climbMotor.setControl(
            motionMagicControl
                .withPosition(setpointRotations * motorReduction)
                // .withFeedForward(feedforward)
                .withSlot(0));

        // if (goalPositionRotations == 0 && positionRotations.get(0).getValueAsDouble() < 0.5) {
        //   climbMotor.setControl(neutralOut);
        // }
    }
}
