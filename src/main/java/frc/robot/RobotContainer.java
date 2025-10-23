// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.CoralLevels;
import frc.robot.FieldConstants.PoleSide;
import frc.robot.commands.ControllerRumbleOnce;
import frc.robot.commands.alignment.AlignToPole;
import frc.robot.commands.states.SetRobotStates;
import frc.robot.commands.states.SetRobotStates.BallIntakeState;
import frc.robot.commands.states.SetRobotStates.CoralIntakeState;
import frc.robot.commands.states.SetRobotStates.RobotState;
import frc.robot.commands.subsystems.DriveCommands;
import frc.robot.commands.subsystems.SetLEDs;
import frc.robot.commands.subsystems.SpinClawIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.clawCanRange.ClawCanRange;
import frc.robot.subsystems.clawIntake.ClawIntake;
import frc.robot.subsystems.clawIntake.ClawIntakeIO;
import frc.robot.subsystems.clawIntake.ClawIntakeIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.climb.Climb.ClimbGoal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.frontIntake.FrontIntake;
import frc.robot.subsystems.frontIntake.FrontIntakeIO;
import frc.robot.subsystems.frontIntake.FrontIntakeIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intakeWrist.IntakeWristIO;
import frc.robot.subsystems.intakeWrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.leds.LedConstants.LEDStates;
import frc.robot.subsystems.rearIntake.RearIntake;
import frc.robot.subsystems.rearIntake.RearIntakeIO;
import frc.robot.subsystems.rearIntake.RearIntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

public class RobotContainer {
  // Subsystems
  public static Drive drive;
  public static Vision vision;
  public static Elevator elevator;
  public static ClawIntake clawIntake;
  public static ClawCanRange clawCanRange;
  public static Climb climb;
  public static FrontIntake frontIntake;
  public static RearIntake rearIntake;
  public static Indexer indexer;
  public static IntakeWrist intakeWrist;
  public static Arm arm;
  public static Led led;

  Optional<Alliance> AllianceColor;

  // State stuff
  public static CoralIntakeState currentCoralIntakeState;
  public static BallIntakeState currentBallIntakeState;

  // Controller
  public static final CommandXboxController Controller = new CommandXboxController(0);
  public static final CommandJoystick Buttons = new CommandJoystick(1);

  // Dashboard inputs
  public final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    currentCoralIntakeState = CoralIntakeState.NotCoralIntaking;
    currentBallIntakeState = BallIntakeState.NotBallIntaking;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
          new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        vision = new Vision(
          drive::addVisionMeasurement,
            new VisionIOLimelight(camera0Name, drive::getRotation),
            new VisionIOLimelight(camera1Name, drive::getRotation)
          );
        elevator = new Elevator(new ElevatorIOTalonFX());
        clawIntake = new ClawIntake(new ClawIntakeIOTalonFX());
        clawCanRange = new ClawCanRange();
        climb = new Climb(new ClimbIOTalonFX());
        led = new Led();
        arm = new Arm(new ArmIOTalonFX());
        frontIntake = new FrontIntake(new FrontIntakeIOTalonFX());
        rearIntake = new RearIntake(new RearIntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
          new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        vision =
          new Vision(
            drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO(){});
        clawIntake = new ClawIntake(new ClawIntakeIO(){});
        clawCanRange = new ClawCanRange();
        climb = new Climb(new ClimbIO(){});
        led = new Led();
        arm = new Arm(new ArmIO(){});
        frontIntake = new FrontIntake(new FrontIntakeIO(){});
        rearIntake = new RearIntake(new RearIntakeIO(){});
        indexer = new Indexer(new IndexerIO(){});
        intakeWrist = new IntakeWrist(new IntakeWristIO(){});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
          new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO(){});
        break;
    }
  
    // Configure named commands
    configNamedCommands();
    
    //** Auto Chooser Routines *//

    // auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // SysId routines
    autoChooser.addDefaultOption(
      "Do Nothing", Commands.none());
    autoChooser.addOption(
      "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
      "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
      "Drive SysId (Quasistatic Forward)",
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
      "Drive SysId (Quasistatic Reverse)",
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
      "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
      "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -Controller.getLeftY(),
            () -> -Controller.getLeftX(),
            () -> -Controller.getRightX()));

    // Tare heading or translation with D-pad
    Controller.povDown()
      .onTrue(Commands.runOnce(() -> drive.tareRotation(),drive).ignoringDisable(true));
    Controller.povUp()
      .onTrue(Commands.runOnce(() -> drive.tareTranslation(),drive).ignoringDisable(true));
  }

  public void teleopTriggers() { // only work in teleop not auto
    Trigger objectInClaw = new Trigger(() -> clawCanRange.objectInClaw);
    Trigger isCoralIntaking = new Trigger(() -> currentCoralIntakeState.equals(CoralIntakeState.CoralIntaking));
    Trigger isAlgaeIntaking = new Trigger(() -> currentBallIntakeState.equals(BallIntakeState.BallIntaking));

    Trigger withinDistancePrep = new Trigger(() -> drive.distanceToTag() <= 1.5);//meters
    Trigger withinDistanceToScore = new Trigger(() -> drive.distanceToTag() <= 0.5);//meters
    Trigger endgame = new Trigger(() -> DriverStation.getMatchTime() <= 15);

    isCoralIntaking.and(objectInClaw).onTrue(new SetRobotStates(RobotState.Default));
    isAlgaeIntaking.and(objectInClaw).onTrue(new SetRobotStates(RobotState.AlgDeafult));

    (Controller.a().or(Controller.b())).and(withinDistancePrep).and(() -> elevator.getCurrentLevel() == CoralLevels.L4).whileTrue(new SetRobotStates(RobotState.PrepScoreL4));
    (Controller.a().or(Controller.b())).and(withinDistancePrep).and(() -> elevator.getCurrentLevel() == CoralLevels.L3).whileTrue(new SetRobotStates(RobotState.PrepScoreL3));
    (Controller.a().or(Controller.b())).and(withinDistancePrep).and(() -> elevator.getCurrentLevel() == CoralLevels.L2).whileTrue(new SetRobotStates(RobotState.PrepScoreL2));

    Controller.a().or(Controller.b()).whileTrue(led.setLedCommand(LEDStates.NotInScoringPosition)); 
    Controller.a().or(Controller.b()).and(withinDistanceToScore).onTrue(new ControllerRumbleOnce(0.25, 0.25, Controller, RumbleType.kBothRumble).alongWith(new SetLEDs(led, LEDStates.InScoringPosition)));

    endgame.onTrue(new InstantCommand(()->climb.getSetpointCommand(ClimbGoal.OUT)));
    endgame.onTrue(new InstantCommand(()->led.setLedCommand(LEDStates.Endgame)));
    endgame.onTrue(
      new ParallelCommandGroup(
        climb.getSetpointCommand(ClimbGoal.OUT),
        led.setLedCommand(LEDStates.Endgame),
        new InstantCommand(()->Elastic.selectTab("Endgame")),
        new InstantCommand(()->Elastic.sendNotification(
          new Elastic.Notification(
            NotificationLevel.WARNING, "Endgame Started", "Climb activated and 15 seconds left in the match")
          )
        )
      )
    );

    Controller.a().whileTrue(new AlignToPole(PoleSide.Left, drive));
    Controller.b().whileTrue(new AlignToPole(PoleSide.Right, drive));

    Controller.rightTrigger().and(() -> elevator.getCurrentLevel() == CoralLevels.L4).onTrue(new SetRobotStates(RobotState.ScoringL4)).onFalse(new SetRobotStates(RobotState.Return));
    Controller.rightTrigger().and(() -> elevator.getCurrentLevel() == CoralLevels.L3).onTrue(new SetRobotStates(RobotState.ScoringL3)).onFalse(new SetRobotStates(RobotState.Return));
    Controller.rightTrigger().and(() -> elevator.getCurrentLevel() == CoralLevels.L2).onTrue(new SetRobotStates(RobotState.ScoringL2)).onFalse(new SetRobotStates(RobotState.Return));
    Controller.rightTrigger().and(() -> elevator.getCurrentLevel() == CoralLevels.L1).onTrue(new SetRobotStates(RobotState.ScoringL1)).onFalse(new SetRobotStates(RobotState.Return));
    Controller.leftTrigger().onTrue(new SetRobotStates(RobotState.Intaking)).onFalse(new SetRobotStates(RobotState.Indexing));
    Controller.rightBumper().onTrue(new SetRobotStates(RobotState.Outtaking)).onFalse(new SetRobotStates(RobotState.Default));

    Controller.button(4).onTrue(new SpinClawIntake(clawIntake, -0.75));
    Controller.button(3).onTrue(new SpinClawIntake(clawIntake, 1)).onFalse(new SpinClawIntake(clawIntake,0));
    Controller.leftBumper().onTrue(new SetRobotStates(RobotState.IntakeL1)).onFalse(new SetRobotStates(RobotState.PrepIntakeL1));

    Buttons.button(12).onTrue(new SetRobotStates(RobotState.PrepScoreL1));
    Buttons.button(11).onTrue(new SetRobotStates(RobotState.SmartScoreL2));
    Buttons.button(2).onTrue(new SetRobotStates(RobotState.SmartScoreL3));
    Buttons.button(1).onTrue(new SetRobotStates(RobotState.SmartScoreL4));

    Buttons.button(10).onTrue(new SetRobotStates(RobotState.AlgDeafult));
    Buttons.button(9).onTrue(new SetRobotStates(RobotState.AlgaeLow));
    Buttons.button(8).onTrue(new SetRobotStates(RobotState.AlgaeHigh));

    Buttons.button(6).onTrue(new SetRobotStates(RobotState.Processor));

    Buttons.button(7).whileTrue(new SetRobotStates(RobotState.PrepThrow)).onFalse(new SetRobotStates(RobotState.ThrowAlgae));

    Buttons.button(5).onTrue(new SetRobotStates(RobotState.PrepClimb));
    Buttons.button(4).onTrue(new SetRobotStates(RobotState.Climbing));

    Buttons.button(3).onTrue(new SetRobotStates(RobotState.BackwardBarge));
  }

  private void configNamedCommands() {
    NamedCommands.registerCommand("Return", new SetRobotStates(RobotState.Return));
    NamedCommands.registerCommand("PrepL4", new SetRobotStates(RobotState.PrepScoreL4Auto));
    NamedCommands.registerCommand("IntakeStart", new SetRobotStates(RobotState.Intaking));
    NamedCommands.registerCommand("IndexerStart", new SetRobotStates(RobotState.IndexingAuto));
    NamedCommands.registerCommand("ScoreL4Smooth", new SetRobotStates(RobotState.ScoringL4AutoSmooth));
    NamedCommands.registerCommand("ScoreL4Rough", new SetRobotStates(RobotState.ScoringL4AutoRough));
    NamedCommands.registerCommand("A1Prep", new SetRobotStates(RobotState.AlgaeLowAuto));
    NamedCommands.registerCommand("A2Prep", new SetRobotStates(RobotState.AlgaeHighAuto));
    NamedCommands.registerCommand("ClawOuttake", new SpinClawIntake(clawIntake, 1));
    NamedCommands.registerCommand("ClawIntake", new SpinClawIntake(clawIntake, -0.25));
    NamedCommands.registerCommand("ClawStop", new SpinClawIntake(clawIntake, 0));
    NamedCommands.registerCommand("BargePrep", new SetRobotStates(RobotState.BackwardBarge));
    NamedCommands.registerCommand("Default", new SetRobotStates(RobotState.Default));
    NamedCommands.registerCommand("AlgaeDefault", new SetRobotStates(RobotState.AlgDeafult));
    NamedCommands.registerCommand("ThrowPrep", new SetRobotStates(RobotState.PrepThrow));
    NamedCommands.registerCommand("ThrowAlgae", new SetRobotStates(RobotState.ThrowAlgae));
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
