package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants.CoralLevels;
import frc.robot.RobotContainer;
import frc.robot.commands.states.SetRobotStates.CoralIntakeState;
import frc.robot.commands.subsystems.RunArm;
import frc.robot.commands.subsystems.RunClimb;
import frc.robot.commands.subsystems.RunElevator;
import frc.robot.commands.subsystems.RunIntakeWrist;
import frc.robot.commands.subsystems.SetLEDs;
import frc.robot.commands.subsystems.SpinClawIntake;
import frc.robot.commands.subsystems.SpinFrontIntake;
import frc.robot.commands.subsystems.SpinIndexer;
import frc.robot.commands.subsystems.SpinRearIntake;
import frc.robot.subsystems.arm.Arm.ArmGoal;
import frc.robot.subsystems.climb.Climb.ClimbGoal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.intakeWrist.IntakeWrist.IntakeWristGoal;
import frc.robot.subsystems.leds.LedConstants.LEDStates;

public class RobotStateCommands {
    public static Command defaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.UP),
                new SpinIndexer(RobotContainer.indexer, 0),
                new SpinClawIntake(RobotContainer.clawIntake, -0.08),
                new SpinFrontIntake(RobotContainer.frontIntake, 0),
                new SpinRearIntake(RobotContainer.rearIntake, 0),
                new RunElevator(RobotContainer.elevator, ElevatorGoal.DEFAULT),
                new RunArm(RobotContainer.arm, ArmGoal.DEFAULT,200),
                new SetLEDs(RobotContainer.led, LEDStates.Default),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.NotCoralIntaking)
            )
        );
    }

    public static Command algDefaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.UP),
                new SpinIndexer(RobotContainer.indexer, 0),
                new SpinClawIntake(RobotContainer.clawIntake, -0.75),
                new SpinFrontIntake(RobotContainer.frontIntake, 0),
                new SpinRearIntake(RobotContainer.rearIntake, 0),
                new RunElevator(RobotContainer.elevator, ElevatorGoal.DEFAULT),
                new RunArm(RobotContainer.arm, ArmGoal.DEFAULT,200),
                new SetLEDs(RobotContainer.led, LEDStates.Default),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.NotCoralIntaking)
            )
        );
    }
    public static Command defaultStateAuto() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinIndexer(RobotContainer.indexer, 0),
                new SpinClawIntake(RobotContainer.clawIntake, -0.03),
                new SpinFrontIntake(RobotContainer.frontIntake, 0),
                new SpinRearIntake(RobotContainer.rearIntake, 0),
                new RunElevator(RobotContainer.elevator, ElevatorGoal.DEFAULT),
                new RunArm(RobotContainer.arm, ArmGoal.DEFAULT,200),
                new SetLEDs(RobotContainer.led, LEDStates.Default),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.NotCoralIntaking)
            )
        );
    }

    public static Command intakingState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.INTAKE),
            new SpinFrontIntake(RobotContainer.frontIntake, 1),
            new SpinRearIntake(RobotContainer.rearIntake, 1),
            new SpinIndexer(RobotContainer.indexer, 0.5),
            new RunArm(RobotContainer.arm, ArmGoal.DEFAULT, 200),
            new RunElevator(RobotContainer.elevator, ElevatorGoal.INTAKE),
            new SetLEDs(RobotContainer.led, LEDStates.Intaking),
            new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.CoralIntaking)
        );
    }

    public static Command L1intake() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.INTAKE),
            new SpinRearIntake(RobotContainer.rearIntake, 0.7),
            new SpinFrontIntake(RobotContainer.frontIntake, -0.03)
        );
    }

    public static Command indexingState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinIndexer(RobotContainer.indexer, -1),
                new SpinClawIntake(RobotContainer.clawIntake, -1),
                new SpinFrontIntake(RobotContainer.frontIntake, 1),
                new SpinRearIntake(RobotContainer.rearIntake, 1),
                new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.UP),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.CoralIntaking)
            )
        );
    }
    public static Command indexingStateAuto() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new WaitCommand(0.8), 
                new SpinIndexer(RobotContainer.indexer, -1),
                new SpinClawIntake(RobotContainer.clawIntake, -1),
                new SpinFrontIntake(RobotContainer.frontIntake, 1),
                new SpinRearIntake(RobotContainer.rearIntake, 1),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.CoralIntaking)
            ),
            defaultStateAuto()
        );
    }

    public static Command prepCoralScore(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new RunArm(RobotContainer.arm, ArmGoal.PREPL2, 2),
                    new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        new RunElevator(RobotContainer.elevator, ElevatorGoal.PREPL2),
                        new SpinClawIntake(RobotContainer.clawIntake, 0),
                        new SpinIndexer(RobotContainer.indexer, 0),
                        new SpinFrontIntake(RobotContainer.frontIntake, 0),
                        new SpinRearIntake(RobotContainer.rearIntake, 0)     
                    )
                );
            case L3:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new RunElevator(RobotContainer.elevator, ElevatorGoal.PREPL3),
                        new SpinClawIntake(RobotContainer.clawIntake, 0)           
                    ),
                    new RunArm(RobotContainer.arm, ArmGoal.PREPL3, 2)
                );
            case L4:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.elevator, ElevatorGoal.PREPL4),
                        new SpinClawIntake(RobotContainer.clawIntake, 0)       
                    ),
                    new RunArm(RobotContainer.arm, ArmGoal.PREPL4, 2)
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.elevator, ElevatorGoal.PREPL4),
                        new SpinClawIntake(RobotContainer.clawIntake, 0),
                        new RunArm(RobotContainer.arm, ArmGoal.PREPL4, 2)
                    )
                );            
            default:
                //L1
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.elevator, ElevatorGoal.PREPL1),
                        new SpinClawIntake(RobotContainer.clawIntake, 0),           
                        new SpinIndexer(RobotContainer.indexer, 0),
                        new SpinFrontIntake(RobotContainer.frontIntake, 0),
                        new SpinRearIntake(RobotContainer.rearIntake, 0)
                    ),
                    new RunArm(RobotContainer.arm, ArmGoal.PREPL1, 2)
                    );   
        }
    }

    public static Command returnState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.UP),
            new SpinFrontIntake(RobotContainer.frontIntake, 0),
            new SpinIndexer(RobotContainer.indexer, 0),
            new SpinClawIntake(RobotContainer.clawIntake, 0),
            new SpinRearIntake(RobotContainer.rearIntake, 0),
            new RunArm(RobotContainer.arm, ArmGoal.RETURN, 200),
            new RunElevator(RobotContainer.elevator, ElevatorGoal.DEFAULT),
            new SetLEDs(RobotContainer.led, LEDStates.Default)
        );
    }

    public static Command scoringState(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new RunElevator(RobotContainer.elevator, ElevatorGoal.SCOREL2),
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new RunArm(RobotContainer.arm, ArmGoal.RETURN, 0.9)
                    )
                );
            case L3:
                return new SequentialCommandGroup(
                    new RunElevator(RobotContainer.elevator, ElevatorGoal.SCOREL3),
                    new RunArm(RobotContainer.arm, ArmGoal.RETURN, 0.9)
                );
            case L4: 
                return new SequentialCommandGroup(
                    new RunElevator(RobotContainer.elevator, ElevatorGoal.SCOREL4),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunArm(RobotContainer.arm, ArmGoal.RETURN, 0.95)
                    )
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.elevator, ElevatorGoal.SCOREL4),
                        new RunArm(RobotContainer.arm, ArmGoal.SCOREL4AUTO, 20)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.2),
                        new SpinClawIntake(RobotContainer.clawIntake, 1)
                    )
                );
            default:
            //L1
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinClawIntake(RobotContainer.clawIntake, 0.25)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinClawIntake(RobotContainer.clawIntake, 0),
                        new RunArm(RobotContainer.arm, ArmGoal.RETURN, 200)
                    ),
                    new RunElevator(RobotContainer.elevator, ElevatorGoal.PREPL2)
                );
        }
    }

    public static Command smartScoringState(CoralLevels level) {
        return new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator);
    }

    public static Command algaeIntakingL2() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.elevator, ElevatorGoal.ALGAEL2),
            new RunArm(RobotContainer.arm, ArmGoal.ALGAEINTAKE, 200),
            new SpinClawIntake(RobotContainer.clawIntake, -0.75)
        );
    }

    public static Command algaeIntakingL1() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.elevator, ElevatorGoal.ALGAEL1),
            new RunArm(RobotContainer.arm, ArmGoal.ALGAEINTAKE, 200),
            new SpinClawIntake(RobotContainer.clawIntake, -0.75)
        );
    }

    public static Command bargeScore() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.elevator, ElevatorGoal.BARGESCORE),
            new RunArm(RobotContainer.arm, ArmGoal.BARGESCORE, 200)
        );
    }

    public static Command prepClimb() {
        return new ParallelCommandGroup(
            // new RunArm(RobotContainer.arm, Constants.Setpoints.Rotator.rotPrepClimb, 200),
            new RunClimb(RobotContainer.climb, ClimbGoal.OUT)
            // new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.ClimbPrep)
            // new RunElevator(RobotContainer.elevator, Constants.Setpoints.Elevator.rElePrepL2)
        );
    }

    public static Command Climb() {
        return new ParallelCommandGroup(
            new RunArm(RobotContainer.arm, ArmGoal.CLIMB, 200),
            new RunClimb(RobotContainer.climb, ClimbGoal.IN),
            new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.CLIMBPREP),
            new RunElevator(RobotContainer.elevator, ElevatorGoal.CLIMB)
        );
    }

    public static Command processorState() {
        return new ParallelCommandGroup(
            new RunArm(RobotContainer.arm, ArmGoal.PROCESSOR, 1),
            new RunElevator(RobotContainer.elevator, ElevatorGoal.PROCESSOR)
        );
    }

    public static Command outtakeState() {
        return new SequentialCommandGroup(
            new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.OUTTAKE),
            new ParallelCommandGroup(
                new SpinFrontIntake(RobotContainer.frontIntake, -0.4),
                new SpinIndexer(RobotContainer.indexer, 0.5),
                new SpinRearIntake(RobotContainer.rearIntake, -0.25)
            )
        );
    }

    public static Command prepIntakeL1() {
        return new SequentialCommandGroup(
            new RunIntakeWrist(RobotContainer.intakeWrist, IntakeWristGoal.OUTTAKE),
            new ParallelCommandGroup(
                new SpinRearIntake(RobotContainer.rearIntake, 0.015)
            )
        );
    }

    public static Command backwardBargeState() {
        return new ParallelCommandGroup(
            new RunArm(RobotContainer.arm, ArmGoal.BACKWARDBARGE, 0.9),
            new RunElevator(RobotContainer.elevator, ElevatorGoal.ALGAETHROW)
        );
    }

    public static Command prepAlgaeThrow() {
        return new ParallelCommandGroup(
                new RunElevator(RobotContainer.elevator, ElevatorGoal.ALGAETHROW),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.7),
                    new RunArm(RobotContainer.arm, ArmGoal.BARGETHROWPREP, 1)
                ),
                new ParallelRaceGroup(
                    new SpinClawIntake(RobotContainer.clawIntake, -0.75),
                    new WaitCommand(0.01)
                )
        );

    }
    public static Command throwAlgae() {
        return new SequentialCommandGroup(
            new RunArm(RobotContainer.arm, ArmGoal.BARGETHROWRELEASE, 0.95),
            new ParallelRaceGroup(
                new SpinClawIntake(RobotContainer.clawIntake, 1),
                new WaitCommand(1)
            )    
        );
    }
}
