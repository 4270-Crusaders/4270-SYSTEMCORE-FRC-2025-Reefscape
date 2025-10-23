package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants.CoralLevels;

public class SetRobotStates extends SequentialCommandGroup {
    public static enum CoralIntakeState {
        CoralIntaking,
        NotCoralIntaking
    }

    public static enum BallIntakeState {
        BallIntaking,
        NotBallIntaking
    }

    public static enum RobotState {
        Default,
        AlgDeafult,
        Indexing,
        IndexingAuto,
        Intaking,
        IntakingAuto,
        Outtaking,

        PrepScoreL1,
        PrepScoreL2,
        PrepScoreL3,
        PrepScoreL4,
        PrepScoreL4Auto,

        ScoringL1,
        ScoringL2,
        ScoringL3,
        ScoringL4,
        ScoringL4AutoSmooth,
        ScoringL4AutoRough,

        PrepClimb,
        Climbing,

        Return,
        AlgaeHigh,
        AlgaeHighAuto,
        AlgaeLow,
        AlgaeLowAuto,
        BargeScore,
        Processor,
        BackwardBarge,
        IntakeL1,
        SmartScoreL1,
        SmartScoreL2,
        SmartScoreL3,
        SmartScoreL4,
        PrepThrow,
        ThrowAlgae,
        PrepIntakeL1
    }

    public SetRobotStates(RobotState State){
        switch (State) {
            case IntakeL1:
                addCommands(
                    RobotStateCommands.L1intake()
                );
                break;
            case Default:      
                addCommands(
                    RobotStateCommands.defaultState()
                );
            case AlgDeafult:
                addCommands(
                    RobotStateCommands.algDefaultState()
                );
                break;
            case Intaking:
                addCommands(
                    RobotStateCommands.intakingState()  
                );
                break;
            case IntakingAuto:
                addCommands(
                    RobotStateCommands.intakingState()  
                );
                break;
            case Outtaking:
                addCommands(
                    RobotStateCommands.outtakeState()
                );
                break;
            case PrepScoreL1:
                addCommands(
                    RobotStateCommands.prepCoralScore(CoralLevels.L1)
                );
                break;
            case PrepScoreL2:
                addCommands(
                    RobotStateCommands.prepCoralScore(CoralLevels.L2)
                );
                break;
            case PrepScoreL3:
                addCommands(
                    RobotStateCommands.prepCoralScore(CoralLevels.L3)
                );
                break;
            case PrepScoreL4:
                addCommands(
                    RobotStateCommands.prepCoralScore(CoralLevels.L4)
                );
                break;
            case PrepScoreL4Auto:
                addCommands(
                    RobotStateCommands.prepCoralScore(CoralLevels.L4)
                );
                break;
            case ScoringL4AutoSmooth:
                addCommands(
                    RobotStateCommands.scoringState(CoralLevels.L4AutoSmooth)
                );
                break;
            case ScoringL4AutoRough:
                addCommands(
                    RobotStateCommands.scoringState(CoralLevels.L4AutoRough)
                );
                break;
            case ScoringL4:
                addCommands(
                    RobotStateCommands.scoringState(CoralLevels.L4)
                );
                break;
            case ScoringL3:
                addCommands(
                    RobotStateCommands.scoringState(CoralLevels.L3)
                );
                break;

            case ScoringL2:
                addCommands(
                    RobotStateCommands.scoringState(CoralLevels.L2)
                );
                break;
            case ScoringL1:
                addCommands(
                    RobotStateCommands.scoringState(CoralLevels.L1)
                );
                break;
            case PrepClimb:
                addCommands(
                    RobotStateCommands.prepClimb()
                );
                break;
            case Climbing:
                addCommands(
                    RobotStateCommands.Climb()
                );
                break;
            case Indexing:
                addCommands(
                    RobotStateCommands.indexingState()
                );
                break;
            case IndexingAuto:
                addCommands(
                    RobotStateCommands.indexingStateAuto()
                );
                break;
            case Return:
                addCommands(
                    RobotStateCommands.returnState()
                );
                break;
            case AlgaeHigh:
                addCommands(
                    RobotStateCommands.algaeIntakingL2()
                );
                break;
            case AlgaeLow:
                addCommands(
                    RobotStateCommands.algaeIntakingL1()
                );
                break;
            case AlgaeHighAuto:
                addCommands(
                    RobotStateCommands.algaeIntakingL2Auto()
                );
                break;
            case AlgaeLowAuto:
                addCommands(
                    RobotStateCommands.algaeIntakingL1Auto()
                );
                break;
            case BargeScore:
                addCommands(
                    RobotStateCommands.bargeScore()
                );
                break;
            case Processor:
                addCommands(
                    RobotStateCommands.processorState()
                );
                break;
            case BackwardBarge:
                addCommands(
                    RobotStateCommands.backwardBargeState()
                );
                break;
            case SmartScoreL1:
                addCommands(
                    RobotStateCommands.smartScoringState(CoralLevels.L1)
                );
                break;
            case SmartScoreL2:
                addCommands(
                    RobotStateCommands.smartScoringState(CoralLevels.L2)
                );
                break;
            case SmartScoreL3:
                addCommands(
                    RobotStateCommands.smartScoringState(CoralLevels.L3)
                );
                break;
            case SmartScoreL4:
                addCommands(
                    RobotStateCommands.smartScoringState(CoralLevels.L4)
                );
                break;
            case ThrowAlgae:
                addCommands(
                    RobotStateCommands.throwAlgae()
                );
                break;
            case PrepThrow:
                addCommands(
                    RobotStateCommands.prepAlgaeThrow()
                );
                break;
            case PrepIntakeL1:
                addCommands(
                    RobotStateCommands.prepIntakeL1()
                );
        }
    }
}
