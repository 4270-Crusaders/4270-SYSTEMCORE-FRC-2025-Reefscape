package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants.CoralLevels;
import frc.robot.RobotContainer;
import frc.robot.commands.states.SetRobotStates.CoralIntakeState;
import frc.robot.commands.subsystems.SetLEDs;
import frc.robot.commands.subsystems.SpinClawIntake;
import frc.robot.commands.subsystems.SpinFrontIntake;
import frc.robot.commands.subsystems.SpinIndexer;
import frc.robot.commands.subsystems.SpinRearIntake;
import frc.robot.subsystems.arm.Arm.ArmGoal;
import frc.robot.subsystems.climb.Climb.ClimbGoal;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.intakeWrist.IntakeWrist.IntakeWristGoal;
import frc.robot.subsystems.leds.LedConstants.LEDStates;

public class RobotStateCommands {
    public static Command defaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.UP),
                RobotContainer.elevator.getSetpointCommand(ElevatorGoal.DEFAULT),
                RobotContainer.arm.getSetpointCommand(ArmGoal.DEFAULT,200),
                new SpinIndexer(RobotContainer.indexer, 0),
                new SpinClawIntake(RobotContainer.clawIntake, -0.02),
                new SpinFrontIntake(RobotContainer.frontIntake, 0),
                new SpinRearIntake(RobotContainer.rearIntake, 0),
                new SetLEDs(RobotContainer.led, LEDStates.Default),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.NotCoralIntaking)
            )
        );
    }

    public static Command algDefaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.UP),
                RobotContainer.elevator.getSetpointCommand(ElevatorGoal.DEFAULT),
                RobotContainer.arm.getSetpointCommand(ArmGoal.DEFAULT,200),
                new SpinIndexer(RobotContainer.indexer, 0),
                new SpinClawIntake(RobotContainer.clawIntake, -0.10),
                new SpinFrontIntake(RobotContainer.frontIntake, 0),
                new SpinRearIntake(RobotContainer.rearIntake, 0),
                new SetLEDs(RobotContainer.led, LEDStates.Default),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.NotCoralIntaking)
            )
        );
    }
    public static Command defaultStateAuto() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RobotContainer.elevator.getSetpointCommand(ElevatorGoal.DEFAULT),
                RobotContainer.arm.getSetpointCommand(ArmGoal.DEFAULT,200),  
                new SpinIndexer(RobotContainer.indexer, 0),
                new SpinClawIntake(RobotContainer.clawIntake, -0.02),
                new SpinFrontIntake(RobotContainer.frontIntake, 0),
                new SpinRearIntake(RobotContainer.rearIntake, 0),
                new SetLEDs(RobotContainer.led, LEDStates.Default),
                new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.NotCoralIntaking)
            )
        );
    }

    public static Command intakingState() {
        return new ParallelCommandGroup(
            RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.INTAKE),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.DEFAULT),
            RobotContainer.arm.getSetpointCommand(ArmGoal.INTAKE,200),  
            new SpinFrontIntake(RobotContainer.frontIntake, 1),
            new SpinRearIntake(RobotContainer.rearIntake, 1),
            new SpinIndexer(RobotContainer.indexer, 0.5),
            new SetLEDs(RobotContainer.led, LEDStates.Intaking),
            new InstantCommand(() -> RobotContainer.currentCoralIntakeState = CoralIntakeState.CoralIntaking)
        );
    }

    public static Command L1intake() {
        return new ParallelCommandGroup(
            RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.INTAKE),
            new SpinFrontIntake(RobotContainer.frontIntake, 0.7),
            new SpinRearIntake(RobotContainer.rearIntake, -0.03)
        );
    }

    public static Command indexingState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinIndexer(RobotContainer.indexer, -1),
                new SpinClawIntake(RobotContainer.clawIntake, -1),
                new SpinFrontIntake(RobotContainer.frontIntake, 1),
                new SpinRearIntake(RobotContainer.rearIntake, 1),
                RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.UP),
                RobotContainer.elevator.getSetpointCommand(ElevatorGoal.INDEX),
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
                    RobotContainer.arm.getSetpointCommand(ArmGoal.PREPL2,2),  
                    new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PREPL2),
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
                        RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PREPL3),
                        new SpinClawIntake(RobotContainer.clawIntake, 0)           
                    ),
                    RobotContainer.arm.getSetpointCommand(ArmGoal.PREPL3,2)
                );
            case L4:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PREPL4),
                        new SpinClawIntake(RobotContainer.clawIntake, 0)       
                    ),
                    RobotContainer.arm.getSetpointCommand(ArmGoal.PREPL4,2)
                );        
            default:
                //L1
                return new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PREPL1),
                        new SpinClawIntake(RobotContainer.clawIntake, 0),           
                        new SpinIndexer(RobotContainer.indexer, 0),
                        new SpinFrontIntake(RobotContainer.frontIntake, 0),
                        new SpinRearIntake(RobotContainer.rearIntake, 0)
                    ),
                    RobotContainer.arm.getSetpointCommand(ArmGoal.PREPL1,2)
                    );   
        }
    }

    public static Command returnState() {
        return new ParallelCommandGroup(
            RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.UP),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.DEFAULT),
            RobotContainer.arm.getSetpointCommand(ArmGoal.RETURN,200),
            new SpinFrontIntake(RobotContainer.frontIntake, 0),
            new SpinIndexer(RobotContainer.indexer, 0),
            new SpinClawIntake(RobotContainer.clawIntake, 0),
            new SpinRearIntake(RobotContainer.rearIntake, 0),
            new SetLEDs(RobotContainer.led, LEDStates.Default)
        );
    }

    public static Command scoringState(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    RobotContainer.elevator.getSetpointCommand(ElevatorGoal.SCOREL2),
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        RobotContainer.arm.getSetpointCommand(ArmGoal.RETURN,0.9)
                    )
                );
            case L3:
                return new SequentialCommandGroup(
                    RobotContainer.elevator.getSetpointCommand(ElevatorGoal.SCOREL3),
                    RobotContainer.arm.getSetpointCommand(ArmGoal.RETURN,0.9)
                );
            case L4: 
                return new SequentialCommandGroup(
                    RobotContainer.elevator.getSetpointCommand(ElevatorGoal.SCOREL4),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        RobotContainer.arm.getSetpointCommand(ArmGoal.RETURN,0.95)
                    )
                );
            case L4AutoRough:
                return new SequentialCommandGroup(
                    RobotContainer.elevator.getSetpointCommand(ElevatorGoal.SCOREL4),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        RobotContainer.arm.getSetpointCommand(ArmGoal.RETURN,0.95)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.2),
                        new SpinClawIntake(RobotContainer.clawIntake, 1)
                    )
                );
            case L4AutoSmooth:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        RobotContainer.elevator.getSetpointCommand(ElevatorGoal.SCOREL4),
                        RobotContainer.arm.getSetpointCommand(ArmGoal.SCOREL4AUTO,20)
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
                        RobotContainer.arm.getSetpointCommand(ArmGoal.RETURN,200)
                    ),
                    RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PREPL2)
                );
        }
    }

    public static Command smartScoringState(CoralLevels level) {
        return new InstantCommand(() -> RobotContainer.elevator.setCurrentLevel(level), RobotContainer.elevator);
    }

    public static Command algaeIntakingL2() {
        return new ParallelCommandGroup(
            RobotContainer.arm.getSetpointCommand(ArmGoal.ALGAEINTAKE,200),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.ALGAEL2),
            new SpinClawIntake(RobotContainer.clawIntake, -0.375)
        );
    }

    public static Command algaeIntakingL1() {
        return new ParallelCommandGroup(
            RobotContainer.arm.getSetpointCommand(ArmGoal.ALGAEINTAKE,200),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.ALGAEL1),
            new SpinClawIntake(RobotContainer.clawIntake, -0.375)
        );
    }

    public static Command bargeScore() {
        return new ParallelCommandGroup(
            RobotContainer.arm.getSetpointCommand(ArmGoal.BARGESCORE,200),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.BARGESCORE)
        );
    }

    public static Command prepClimb() {
        return new ParallelCommandGroup(
            RobotContainer.climb.getSetpointCommand(ClimbGoal.OUT)
        );
    }

    public static Command Climb() {
        return new ParallelCommandGroup(
            RobotContainer.arm.getSetpointCommand(ArmGoal.CLIMB,200),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.CLIMB),
            RobotContainer.climb.getSetpointCommand(ClimbGoal.IN),
            RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.CLIMBPREP)
        );
    }

    public static Command processorState() {
        return new ParallelCommandGroup(
            RobotContainer.arm.getSetpointCommand(ArmGoal.PROCESSOR,1),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PROCESSOR)
        );
    }

    public static Command outtakeState() {
        return new SequentialCommandGroup(
            RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.OUTTAKE),
            new ParallelCommandGroup(
                new SpinFrontIntake(RobotContainer.frontIntake, -0.4),
                new SpinIndexer(RobotContainer.indexer, 0.5),
                new SpinRearIntake(RobotContainer.rearIntake, -0.25)
            )
        );
    }

    public static Command prepIntakeL1() {
        return new SequentialCommandGroup(
            RobotContainer.intakeWrist.getSetpointCommand(IntakeWristGoal.OUTTAKE),
            new ParallelCommandGroup(
                new SpinRearIntake(RobotContainer.rearIntake, 0.015)
            )
        );
    }

    public static Command backwardBargeState() {
        return new ParallelCommandGroup(
            RobotContainer.arm.getSetpointCommand(ArmGoal.BACKWARDBARGE,0.9),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.ALGAETHROW)
        );
    }

    public static Command prepAlgaeThrow() {
        return new ParallelCommandGroup(
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.PREPALGAETHROW),
            RobotContainer.arm.getSetpointCommand(ArmGoal.BARGETHROWPREP,1),
            new SpinClawIntake(RobotContainer.clawIntake, -0.10)
        );

    }
    public static Command throwAlgae() {
        return new SequentialCommandGroup(
            new SpinClawIntake(RobotContainer.clawIntake, -0.10),
            RobotContainer.elevator.getSetpointCommand(ElevatorGoal.ALGAETHROW),
            RobotContainer.arm.getSetpointCommand(ArmGoal.BARGETHROWRELEASE,0.95),
            new WaitCommand(0.25),
            new ParallelRaceGroup(
                new SpinClawIntake(RobotContainer.clawIntake, 1),
                new WaitCommand(1)
            )    
        );
    }
}
