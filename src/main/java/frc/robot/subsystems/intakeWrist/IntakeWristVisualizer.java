package frc.robot.subsystems.intakeWrist;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeWristVisualizer {
    private LoggedMechanism2d mechanism;
    private LoggedMechanismLigament2d intakeWrist;
    private String key;
    public IntakeWristVisualizer(String key, Color color) {
        mechanism = new LoggedMechanism2d(2.0, 2.0, new Color8Bit(Color.kWhite));
        intakeWrist = new LoggedMechanismLigament2d("intakeWrist", 1.0, 90, 8, new Color8Bit(color));
        mechanism.getRoot("pivot", 0.5, 0).append(intakeWrist);
        this.key = key;
    }

    public void update(double angle) {
        intakeWrist.setAngle(90+angle);
        Logger.recordOutput("IntakeWrist/Mechanism2d" + key, mechanism);
    }
    
}
