package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.ARM_CAN_ID;
import static frc.robot.subsystems.arm.ArmConstants.ARM_SENSOR_CAB_ID;

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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

public class ArmIOTalonFX implements ArmIO{
    // Hardware
    private final TalonFX armMotor;

    // Status Signals
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private double goalPositionDegrees;

    // Control
    private final NeutralOut neutralOut = new NeutralOut();
    private final PositionVoltage positionControl = new PositionVoltage(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0.0);

    private final MotionMagicExpoVoltage motionMagicControl =
        new MotionMagicExpoVoltage(0.0);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    public ArmIOTalonFX() {
        armMotor = new TalonFX(ARM_CAN_ID);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.MotionMagic.withMotionMagicCruiseVelocity(10);
        config.MotionMagic.withMotionMagicExpo_kV(0.001);
        config.MotionMagic.withMotionMagicExpo_kA(0.00001);
        config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM1).withFeedbackRemoteSensorID(ARM_SENSOR_CAB_ID);

        armMotor.getClosedLoopReference();

        PhoenixUtil.tryUntilOk(5, () -> armMotor.getConfigurator().apply(config));

        positionRotations = armMotor.getPosition();
        velocityRps = armMotor.getVelocity();
        appliedVoltage = armMotor.getMotorVoltage();
        supplyCurrent = armMotor.getSupplyCurrent();
        torqueCurrent = armMotor.getTorqueCurrent();
        tempCelsius = armMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            positionRotations,
            velocityRps,
            appliedVoltage,
            supplyCurrent,
            torqueCurrent,
            tempCelsius);

        goalPositionDegrees = 0.0;
    }
    private static final double ENCODER_OFFSET_ROT = 0.231;
    private static final double ENCODER_OFFSET_DEG = ENCODER_OFFSET_ROT*360;

    public void updateInputs(ArmIOInputs inputs) {
        inputs.MotorConnected =
            BaseStatusSignal.refreshAll(
                    positionRotations,
                    velocityRps,
                    appliedVoltage,
                    supplyCurrent,
                    torqueCurrent,
                    tempCelsius)
                .isOK();

        inputs.positionRotation = positionRotations.getValueAsDouble()+ENCODER_OFFSET_ROT;
        inputs.positionDegrees = Units.rotationsToDegrees(inputs.positionRotation);
        inputs.velocityRotationsPerSecond = velocityRps.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    }

    @Override
    public void stop() {
        armMotor.setControl(neutralOut);
    }

    @Override
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
            armMotor.getPosition().getValueAsDouble(), goalPositionDegrees, 5);
    }

    @Override
    public double getDegrees(){
        return armMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void runVolts(double volts) {
        armMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void runCurrent(double amps) {
        armMotor.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
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

        PhoenixUtil.tryUntilOk(5, () -> armMotor.getConfigurator().apply(config));
    }


    @Override
    public void runSetpoint(double setpointDegrees, double feedforward) {
        double setpointRot = Units.degreesToRotations(setpointDegrees-ENCODER_OFFSET_DEG);
        Logger.recordOutput("Arm/SetpointDegrees", setpointDegrees);

        armMotor.setControl( // kG 0.5 kV 1000.
            positionControl.withPosition(setpointRot).withEnableFOC(true).withFeedForward(feedforward));
    }

    @Override
    public void runSetpointMotionMagic(double setpointDegrees, double feedforward, double maxVelocity) {
        goalPositionDegrees = Units.degreesToRotations(setpointDegrees-ENCODER_OFFSET_DEG);
        Logger.recordOutput("Arm/SetpointDegrees", setpointDegrees);
        armMotor.getConfigurator().apply(new TalonFXConfiguration().MotionMagic.withMotionMagicCruiseVelocity(maxVelocity));
        armMotor.setControl(
            motionMagicControl
                .withPosition(Units.degreesToRotations(setpointDegrees-ENCODER_OFFSET_DEG))
                .withFeedForward(feedforward)
                .withSlot(0)
                .withEnableFOC(true));
    }
}
