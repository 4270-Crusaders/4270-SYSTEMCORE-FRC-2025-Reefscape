package frc.robot.subsystems.intakeWrist;

import static frc.robot.subsystems.intakeWrist.IntakeWristConstants.intakeWristCanId;
import static frc.robot.subsystems.intakeWrist.IntakeWristConstants.motorReduction;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.EqualsUtil;
import frc.robot.util.PhoenixUtil;

public class IntakeWristIOTalonFX implements IntakeWristIO{
    // Hardware
    private final TalonFX intakeWristMotor;

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

    public IntakeWristIOTalonFX() {
        intakeWristMotor = new TalonFX(intakeWristCanId);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.MotionMagic.withMotionMagicCruiseVelocity(250);
        config.MotionMagic.withMotionMagicExpo_kV(0.002);
        config.MotionMagic.withMotionMagicExpo_kA(0.0005);

        intakeWristMotor.getClosedLoopReference();

        PhoenixUtil.tryUntilOk(5, () -> intakeWristMotor.getConfigurator().apply(config));

        positionRotations = intakeWristMotor.getPosition();
        velocityRps = intakeWristMotor.getVelocity();
        appliedVoltage = intakeWristMotor.getMotorVoltage();
        supplyCurrent = intakeWristMotor.getSupplyCurrent();
        torqueCurrent = intakeWristMotor.getTorqueCurrent();
        tempCelsius = intakeWristMotor.getDeviceTemp();
        setpointRotations = intakeWristMotor.getClosedLoopReference();

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

    public void updateInputs(IntakeWristIOInputs inputs) {
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
        inputs.positionDegrees = Units.rotationsToDegrees(inputs.positionRotations/motorReduction);
        inputs.velocityRotationsPerSecond = velocityRps.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.setpointRotations = setpointRotations.getValueAsDouble();
    }

    @Override
    public void stop() {
        intakeWristMotor.setControl(neutralOut);
    }

    @Override
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
            intakeWristMotor.getPosition().getValueAsDouble(), goalPositionRotations, 2.5);
    }

    @Override
    public void runVolts(double volts) {
        intakeWristMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void runCurrent(double amps) {
        intakeWristMotor.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
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

        PhoenixUtil.tryUntilOk(5, () -> intakeWristMotor.getConfigurator().apply(config));
    }


    @Override
    public void runSetpoint(double setpointDegrees, double feedforward) {
        goalPositionRotations = Units.degreesToRotations(setpointDegrees)*motorReduction;
        double setpoint = Units.degreesToRotations(setpointDegrees) * motorReduction;
        Logger.recordOutput("IntakeWrist/SetpointDegrees", setpointDegrees);

        intakeWristMotor.setControl( // kG 0.5 kV 1000.
            positionControl.withPosition(setpoint).withEnableFOC(true).withFeedForward(feedforward));
    }

    @Override
    public void runSetpointMotionMagic(double setpointDegrees, double feedforward) {
        goalPositionRotations = Units.degreesToRotations(setpointDegrees)*motorReduction;
        Logger.recordOutput("IntakeWrist/SetpointDegrees", setpointDegrees);
        intakeWristMotor.setControl(
            motionMagicControl
                .withEnableFOC(true)
                .withPosition(Units.degreesToRotations(setpointDegrees) * motorReduction)
                .withFeedForward(feedforward)
                .withSlot(0));
    }
}
