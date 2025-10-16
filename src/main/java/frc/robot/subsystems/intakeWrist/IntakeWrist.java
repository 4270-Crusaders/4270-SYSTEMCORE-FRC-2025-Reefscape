package frc.robot.subsystems.intakeWrist;

import static frc.robot.subsystems.intakeWrist.IntakeWristConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.profileConstraints;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeWrist extends SubsystemBase {
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("IntakeWrist/maxVelocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("IntakeWrist/maxAcceleration", profileConstraints.maxAcceleration);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakeWrist/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("IntakeWrist/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakeWrist/kD", gains.kD());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("IntakeWrist/kG", gains.ffkG());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("IntakeWrist/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("IntakeWrist/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("IntakeWrist/kA", gains.ffkA());

  public final IntakeWristIO io;
  public final IntakeWristIOInputsAutoLogged inputs = new IntakeWristIOInputsAutoLogged();

  ArmFeedforward feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private final IntakeWristVisualizer setpointVisualizer;
  private final IntakeWristVisualizer goalVisualizer;

  private double goalDegrees;

  public enum IntakeWristGoal {  
    UP(new LoggedTunableNumber("IntakeWrist/Up", 10)),
    OUTTAKE(new LoggedTunableNumber("IntakeWrist/Outtake", 58)),
    INTAKE(new LoggedTunableNumber("IntakeWrist/Intake", 142)),
    CLIMBPREP(new LoggedTunableNumber("IntakeWrist/ClimbPrep", 24.5));

    private final DoubleSupplier intakeWristSetpointSupplier;

    private IntakeWristGoal(DoubleSupplier intakeWristSetpointSupplier) {
      this.intakeWristSetpointSupplier = intakeWristSetpointSupplier;
    }

    private double getDegrees() {
      return intakeWristSetpointSupplier.getAsDouble();
    }
  }


  @AutoLogOutput private IntakeWristGoal goal = IntakeWristGoal.UP;

  boolean closedLoop = true;

  public void setGoal(IntakeWristGoal newGoal) {
    closedLoop = true;
    goal = newGoal;
  }

  public void returnToHome() {
    goal = IntakeWristGoal.UP;
  }

  public void manualCurrent(double amps) {
    closedLoop = false;
    io.runCurrent(amps);
  }

  public void stop() {
    closedLoop = false;
    io.stop();
  }

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    setpointVisualizer = new IntakeWristVisualizer("IntakeWrist/Setpoint", Color.kGreen);
    goalVisualizer = new IntakeWristVisualizer("IntakeWrist/IntakeWristGoal", Color.kRed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeWrist", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kV.get(), kS.get(), kA.get(), kG.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA,
        kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(new Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);

    goalDegrees = goal.getDegrees();

    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs, setpointState, new TrapezoidProfile.State(goalDegrees, 0.0));

    // motion magic setpoint code
    if (closedLoop) {
      io.runSetpointMotionMagic(
        goalDegrees,0);
    }

    Logger.recordOutput("IntakeWrist/SetpointPos", setpointState.position);
    Logger.recordOutput("IntakeWrist/GoalPos", goalDegrees);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goalDegrees);
  }

  public Command getSetpointCommand(IntakeWristGoal goal) {
    return new Command() {
      @Override
      public void initialize() {
        setGoal(goal);
      }

      @Override
      public boolean isFinished() {
        return EqualsUtil.epsilonEquals(goal.getDegrees(), inputs.positionDegrees, 0.3);
      }
    };
  }

  @AutoLogOutput
  public boolean atGoal() {
    return io.atGoal();
  }
}