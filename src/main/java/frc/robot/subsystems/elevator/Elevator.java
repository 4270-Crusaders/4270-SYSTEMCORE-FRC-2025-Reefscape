package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants.CoralLevels;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/maxVelocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/maxAcceleration", profileConstraints.maxAcceleration);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", gains.kD());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/kG", gains.ffkG());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/kA", gains.ffkA());

  // private static final LoggedTunableNumber maxVelocityMotionMagic =
  //     new LoggedTunableNumber("Elevator/MotionMagic/Velocity", motionMagicConstraints.velocity());
  // private static final LoggedTunableNumber maxAccelerationMotionMagic =
  //     new LoggedTunableNumber(
  //         "Elevator/MotionMagic/Acceleration", motionMagicConstraints.acceleration());
  // private static final LoggedTunableNumber maxJerkMotionMagic =
  //     new LoggedTunableNumber("Elevator/MotionMagic/Jerk", motionMagicConstraints.jerk());

  public final ElevatorIO io;
  public final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  ElevatorFeedforward feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private final ElevatorVisualizer setpointVisualizer;
  private final ElevatorVisualizer goalVisualizer;

  private double goalMeters;

  @AutoLogOutput
  private static CoralLevels currentLevel;

  public enum ElevatorGoal {  //(RotationPosition/(175/18))*3.14159625*0.0444754*2 = meterPositon
    DEFAULT(new LoggedTunableNumber("Elevator/DEFAULT", 0.03)),
    STOW(new LoggedTunableNumber("Elevator/Stow", 0.03)),
    INTAKE(new LoggedTunableNumber("Elevator/Intake", 0.02)),

    PREPL1(new LoggedTunableNumber("Elevator/PrepL1", 0.18)),
    PREPL2(new LoggedTunableNumber("Elevator/PrepL2", 0.015)),
    PREPL3(new LoggedTunableNumber("Elevator/PrepL3", 0.4)),
    PREPL4(new LoggedTunableNumber("Elevator/PrepL4", 1.35)),

    SCOREL1(new LoggedTunableNumber("Elevator/ScoreL1", PREPL1.getMeters() - coralScoringDownOffset)),
    SCOREL2(new LoggedTunableNumber("Elevator/ScoreL2", PREPL2.getMeters() - coralScoringDownOffset)),
    SCOREL3(new LoggedTunableNumber("Elevator/ScoreL3", PREPL3.getMeters() - coralScoringDownOffset)),
    SCOREL4(new LoggedTunableNumber("Elevator/ScoreL4", PREPL4.getMeters() - coralScoringDownOffset)),

    ALGAEL1(new LoggedTunableNumber("Elevator/ALGAEL1", 0.15)),
    ALGAEL2(new LoggedTunableNumber("Elevator/ALGAEL2", 0.80)),

    PROCESSOR(new LoggedTunableNumber("Elevator/PrepL2", 0.0175)),

    ALGAEPROCESSOR(new LoggedTunableNumber("Elevator/AlgaeProcessor", 0.0129344271)),
    PREPALGAETHROW(new LoggedTunableNumber("Elevator/PREPALGAETHROW", 1.3)),
    ALGAETHROW(new LoggedTunableNumber("Elevator/ALGAETHROW", 1.3)),

    BARGESCORE(new LoggedTunableNumber("Elevator/BargeScore", 0.54)),

    CLIMB(new LoggedTunableNumber("Elevator/CLIMB", 0.0175));

    private final DoubleSupplier elevatorSetpointSupplier;

    private ElevatorGoal(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    private double getMeters() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput private ElevatorGoal goal = ElevatorGoal.STOW;

  boolean closedLoop = true;

  public void setGoal(ElevatorGoal newGoal) {
    closedLoop = true;
    goal = newGoal;
  }

  public void returnToHome() {
    goal = ElevatorGoal.STOW;
  }

  public void manualCurrent(double amps) {
    closedLoop = false;
    io.runCurrent(amps);
  }

  public void stop() {
    closedLoop = false;
    io.stop();
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    setpointVisualizer = new ElevatorVisualizer("Elevator/Setpoint", Color.kGreen);
    goalVisualizer = new ElevatorVisualizer("Elevator/ElevatorGoal", Color.kRed);
    currentLevel = CoralLevels.L4;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    // Logger.recordOutput("Elevator/CurrentLevel", currentLevel);

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

    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     () ->
    //         io.setMotionMagicConstraints(
    //             maxVelocityMotionMagic.get(),
    //             maxAccelerationMotionMagic.get(),
    //             maxJerkMotionMagic.get()),
    //     maxVelocityMotionMagic,
    //     maxAccelerationMotionMagic,
    //     maxJerkMotionMagic);

    goalMeters = goal.getMeters();

    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs, setpointState, new TrapezoidProfile.State(goalMeters, 0.0));

    // setpoint code, comment out if using motion magic
    // if (closedLoop) {
    //   io.runSetpoint(
    //       setpointState.position,
    //       kG.get()
    //           + kV.get() * setpointState.velocity
    //           + kS.get() * Math.signum(setpointState.velocity));
    // }

    // motion magic setpoint code
    if (closedLoop) {
      io.runSetpointMotionMagic(
          goalMeters,0);
    }

    Logger.recordOutput("Elevator/SetpointPos", setpointState.position);
    Logger.recordOutput("Elevator/GoalPos", goalMeters);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goalMeters);
  }

  public Command getSetpointCommand(ElevatorGoal goal) {
    return new Command() {
      @Override
      public void initialize() {
        setGoal(goal);
      }

      @Override
      public boolean isFinished() {
        return atGoal();
      }
    };
  }

  @AutoLogOutput
  public boolean atGoal() {
    return io.atGoal();
  }

  public void setCurrentLevel(CoralLevels level) {
    currentLevel = level;
  }

  public CoralLevels getCurrentLevel() {
    return currentLevel;
  }

  public void moveToCurrentLevel(){
    switch (currentLevel) {
      case L1:
        setGoal(ElevatorGoal.SCOREL1);
        break;
      case L2:
        setGoal(ElevatorGoal.SCOREL2);
        break;
      case L3:
        setGoal(ElevatorGoal.SCOREL3);
        break;
      case L4:
        setGoal(ElevatorGoal.SCOREL4);
        break;
      default:
        setGoal(ElevatorGoal.STOW);
        break;
    }
  }
}