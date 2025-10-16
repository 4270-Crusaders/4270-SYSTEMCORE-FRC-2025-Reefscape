package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbVisualizer {
    private LoggedMechanism2d mechanism;
    private LoggedMechanismLigament2d climb;
    private String key;
    public ClimbVisualizer(String key, Color color) {
        mechanism = new LoggedMechanism2d(2.0, 2.0, new Color8Bit(Color.kWhite));
        climb = new LoggedMechanismLigament2d("climb", 1.0, 90, 8, new Color8Bit(color));
        mechanism.getRoot("pivot", 0.5, 0).append(climb);
        this.key = key;
    }

    public void update(double angle) {
        climb.setAngle(90+angle);
        Logger.recordOutput("Climb/Mechanism2d" + key, mechanism);
    }
    
}
