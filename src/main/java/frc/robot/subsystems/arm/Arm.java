package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/kG", gains.ffkG());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/kA", gains.ffkA());

  public final ArmIO io;
  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  ArmFeedforward feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  private double goalDegrees;

  public enum ArmGoal { 
    DEFAULT(new LoggedTunableNumber("Arm/Default", 180)),
    BALLDEFAULT(new LoggedTunableNumber("Arm/BallDefault", 202)),
    INTAKE(new LoggedTunableNumber("Arm/Intake", -5)),
    PREPL1(new LoggedTunableNumber("Arm/PrepL1", 65)),
    PREPL2(new LoggedTunableNumber("Arm/PrepL2", 115)),
    PREPL3(new LoggedTunableNumber("Arm/PrepL3", 130)),
    PREPL4(new LoggedTunableNumber("Arm/PrepL4", 140)),
    RETURN(new LoggedTunableNumber("Arm/Return", 0)),
    SCOREL4AUTO(new LoggedTunableNumber("Arm/ScoreL4sAuto", 113.5)),
    ALGAEINTAKE(new LoggedTunableNumber("Arm/AlgaeIntake", 95)),
    BARGETHROWPREP(new LoggedTunableNumber("Arm/BargeThrowPrep", 103.7)),
    BARGETHROWRELEASE(new LoggedTunableNumber("Arm/BargeThrowRelease", 251)),
    CLIMB(new LoggedTunableNumber("Arm/Climb", 90)),
    PROCESSOR(new LoggedTunableNumber("Arm/Processor", 62.5)),

    BARGESCORE(new LoggedTunableNumber("Arm/BargeScore", 178.5)),
    BACKWARDBARGE(new LoggedTunableNumber("Arm/BackwardBarge", 178.5)); //0.65

    private final DoubleSupplier armSetpointSupplier;

    private ArmGoal(DoubleSupplier armSetpointSupplier) {
      this.armSetpointSupplier = armSetpointSupplier;
    }

    private double getDegrees() {
      return armSetpointSupplier.getAsDouble();
    }
  }


  @AutoLogOutput private ArmGoal goal = ArmGoal.RETURN;
  @AutoLogOutput private double maxVelocity = 10.0;

  boolean closedLoop = true;

  public void setGoal(ArmGoal newGoal) {
    closedLoop = true;
    goal = newGoal;
  }

  public void setMaxVelocity(double newMaxVelocity){
    maxVelocity = newMaxVelocity;
  }

  public void returnToHome() {
    goal = ArmGoal.RETURN;
  }

  public void manualCurrent(double amps) {
    closedLoop = false;
    io.runCurrent(amps);
  }

  public void stop() {
    closedLoop = false;
    io.stop();
  }

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

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

    goalDegrees = goal.getDegrees();

    // motion magic setpoint code
    if (closedLoop) {
      io.runSetpointMotionMagic(goalDegrees, 0, maxVelocity);
    }

    Logger.recordOutput("Arm/GoalPos", goalDegrees);
  }

  public Command getSetpointCommand(ArmGoal goal, double maxVelocity) {
    return new Command() {
      @Override
      public void initialize() {
        setMaxVelocity(maxVelocity);
        setGoal(goal);
      }

      @Override
      public boolean isFinished() {
        return EqualsUtil.epsilonEquals(goal.getDegrees(), inputs.positionDegrees, 0.008);
      }
    };
  }

  @AutoLogOutput
  public boolean atGoal() {
    return io.atGoal();
  }
}