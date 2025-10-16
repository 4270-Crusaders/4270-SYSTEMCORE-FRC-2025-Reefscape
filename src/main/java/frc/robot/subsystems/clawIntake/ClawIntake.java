package frc.robot.subsystems.clawIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class ClawIntake extends SubsystemBase {
  private final ClawIntakeIO io;
  private final ClawIntakeIOInputsAutoLogged inputs = new ClawIntakeIOInputsAutoLogged();

  public ClawIntake(ClawIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ClawIntake", inputs);
  }

  public Command runPercentVoltage(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public void runPowerDutyCycleOut(double power) {
    io.runPowerDutyCycleOut(power);
  }
}