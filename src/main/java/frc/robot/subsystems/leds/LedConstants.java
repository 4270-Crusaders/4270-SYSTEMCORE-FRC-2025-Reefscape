package frc.robot.subsystems.leds;

public class LedConstants {
    public static final int ledsID = 61;
    
    public static final int LedEndIndex = 120;

    public enum LEDStates {
        Default,
        Intaking,
        NotInScoringPosition,
        InScoringPosition,
        DisabledRed,
        DisabledBlue,
        Endgame
    }
}
