package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

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

public class Climb extends SubsystemBase {
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Climb/maxVelocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Climb/maxAcceleration", profileConstraints.maxAcceleration);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", gains.kD());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Climb/kG", gains.ffkG());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Climb/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Climb/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Climb/kA", gains.ffkA());

  // private static final LoggedTunableNumber maxVelocityMotionMagic =
  //     new LoggedTunableNumber("Climb/MotionMagic/Velocity", motionMagicConstraints.velocity());
  // private static final LoggedTunableNumber maxAccelerationMotionMagic =
  //     new LoggedTunableNumber(
  //         "Climb/MotionMagic/Acceleration", motionMagicConstraints.acceleration());
  // private static final LoggedTunableNumber maxJerkMotionMagic =
  //     new LoggedTunableNumber("Climb/MotionMagic/Jerk", motionMagicConstraints.jerk());

  public final ClimbIO io;
  public final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  ArmFeedforward feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private final ClimbVisualizer setpointVisualizer;
  private final ClimbVisualizer goalVisualizer;

  private double goalRotation;

  public enum ClimbGoal {  
    OUT(new LoggedTunableNumber("Climb/Out", 220)),
    DEAFULT(new LoggedTunableNumber("Climb/Default", 58)),
    IN(new LoggedTunableNumber("Climb/In", 40));
    // public static final double wristUp = -4;
    //   public static final double wristOuttake = -9.71;
    //   public static final double wristIntake = -24.4;
    //   public static final double wristClimbPrep = -4.1;

    private final DoubleSupplier climbSetpointSupplier;

    private ClimbGoal(DoubleSupplier climbSetpointSupplier) {
      this.climbSetpointSupplier = climbSetpointSupplier;
    }

    private double getDegrees() {
      return climbSetpointSupplier.getAsDouble();
    }
  }


  @AutoLogOutput private ClimbGoal goal = ClimbGoal.DEAFULT;

  boolean closedLoop = true;

  public void setGoal(ClimbGoal newGoal) {
    closedLoop = true;
    goal = newGoal;
  }

  public void returnToHome() {
    goal = ClimbGoal.DEAFULT;
  }

  public void manualCurrent(double amps) {
    closedLoop = false;
    io.runCurrent(amps);
  }

  public void stop() {
    closedLoop = false;
    io.stop();
  }

  public Climb(ClimbIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    setpointVisualizer = new ClimbVisualizer("Climb/Setpoint", Color.kGreen);
    goalVisualizer = new ClimbVisualizer("Climb/ClimbGoal", Color.kRed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

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

        goalRotation = goal.getDegrees();

    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs, setpointState, new TrapezoidProfile.State(goalRotation, 0.0));

    // motion magic setpoint code
    if (closedLoop) {
      io.runSetpointMotionMagic(
        goalRotation,0);
    }

    Logger.recordOutput("Climb/SetpointPos", setpointState.position);
    Logger.recordOutput("Climb/GoalPos", goalRotation);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goalRotation);
  }

  public Command getSetpointCommand(ClimbGoal goal) {
    return new Command() {
      @Override
      public void initialize() {
        setGoal(goal);
      }

      @Override
      public boolean isFinished() {
        return EqualsUtil.epsilonEquals(goal.getDegrees(), inputs.positionRotations, 0.003);
      }
    };
  }

  @AutoLogOutput
  public boolean atGoal() {
    return io.atGoal();
  }
}