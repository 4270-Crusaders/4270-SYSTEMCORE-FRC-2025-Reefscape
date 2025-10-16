package frc.robot.subsystems.rearIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RearIntake extends SubsystemBase {
  private final RearIntakeIO io;
  private final RearIntakeIOInputsAutoLogged inputs = new RearIntakeIOInputsAutoLogged();

  public RearIntake(RearIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RearIntake", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public void runPowerDutyCycleOut(double power) {
    io.runPowerDutyCycleOut(power);
  }
}