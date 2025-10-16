package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorFollowerCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorMainCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.motorReduction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.EqualsUtil;
import frc.robot.util.PhoenixUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX main;
  private final TalonFX follower;

  // Status Signals
  private final List<StatusSignal<Angle>> positionRotations;
  private final List<StatusSignal<AngularVelocity>> velocityRps;
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;
  private final List<StatusSignal<Double>> setpointRotations;

  private double goalPositionRotations;

  // Control
  private final NeutralOut neutralOut = new NeutralOut();
  private final PositionVoltage positionControl = new PositionVoltage(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final Follower followerControl = new Follower(elevatorMainCanId, true);
  // private final PositionTorqueCurrentFOC positionCurrentControl = new PositionTorqueCurrentFOC(0.0);

  private final MotionMagicExpoVoltage motionMagicControl =
      new MotionMagicExpoVoltage(0.0);

  private TalonFXConfiguration config = new TalonFXConfiguration();

  public ElevatorIOTalonFX() {
    main = new TalonFX(elevatorMainCanId);
    follower = new TalonFX(elevatorFollowerCanId);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.MotionMagic.withMotionMagicCruiseVelocity(250);
    config.MotionMagic.withMotionMagicExpo_kV(0.002);
    config.MotionMagic.withMotionMagicExpo_kA(0.0005);
    // config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    main.getClosedLoopReference();

    PhoenixUtil.tryUntilOk(5, () -> main.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));

    follower.setControl(followerControl);

    positionRotations = List.of(main.getPosition(), follower.getPosition());
    velocityRps = List.of(main.getVelocity(), follower.getVelocity());
    appliedVoltage = List.of(main.getMotorVoltage(), follower.getMotorVoltage());
    supplyCurrent = List.of(main.getSupplyCurrent(), follower.getSupplyCurrent());
    torqueCurrent = List.of(main.getTorqueCurrent(), follower.getTorqueCurrent());
    tempCelsius = List.of(main.getDeviceTemp(), follower.getDeviceTemp());
    setpointRotations = List.of(main.getClosedLoopReference(), follower.getClosedLoopReference());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        positionRotations.get(0),
        positionRotations.get(1),
        velocityRps.get(0),
        velocityRps.get(1),
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));
        
    goalPositionRotations = 0.0;
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations.get(0),
                velocityRps.get(0),
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0),
                setpointRotations.get(0))
            .isOK();

    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                velocityRps.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1),
                setpointRotations.get(1))
            .isOK();

    inputs.positionRotations =
        positionRotations.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.velocityMeters =
        velocityRps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.positionMeters =
        new double[] {
          (inputs.positionRotations[0] / motorReduction) * ElevatorConstants.rotationsToMeters,
          (inputs.positionRotations[0] / motorReduction) * ElevatorConstants.rotationsToMeters
        };
    inputs.setpointRotations =
        setpointRotations.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @Override
  public void stop() {
    main.setControl(neutralOut);
  }

  @Override
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        main.getPosition().getValueAsDouble(), goalPositionRotations, 1.0);
  }

  @Override
  public void runVolts(double volts) {
    main.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void runCurrent(double amps) {
    main.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
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

    PhoenixUtil.tryUntilOk(5, () -> main.getConfigurator().apply(config));
  }


  @Override
  public void runSetpoint(double setpointMeters, double feedforward) {
    double setpoint = (setpointMeters / ElevatorConstants.rotationsToMeters) * motorReduction;

    Logger.recordOutput("Elevator/SetpointRotations", setpoint);

    main.setControl( // kG 0.5 kV 1000.
        positionControl.withPosition(setpoint).withEnableFOC(true).withFeedForward(feedforward));
  }

  @Override
  public void runSetpointMotionMagic(double setpointMeters, double feedforward) {
    goalPositionRotations = (setpointMeters / ElevatorConstants.rotationsToMeters) * motorReduction;
    Logger.recordOutput("Elevator/SetpointRotations", goalPositionRotations);
    main.setControl(
        motionMagicControl
          .withEnableFOC(true)
          .withPosition((setpointMeters / ElevatorConstants.rotationsToMeters) * motorReduction)
          // .withFeedForward(feedforward)
          .withSlot(0));

    if (goalPositionRotations == 0 && positionRotations.get(0).getValueAsDouble() < 0.5) {
      main.setControl(neutralOut);
    }
  }
}