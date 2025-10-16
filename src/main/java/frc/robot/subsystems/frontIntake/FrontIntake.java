package frc.robot.subsystems.frontIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FrontIntake extends SubsystemBase {
  private final FrontIntakeIO io;
  private final FrontIntakeIOInputsAutoLogged inputs = new FrontIntakeIOInputsAutoLogged();

  public FrontIntake(FrontIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FrontIntake", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public void runPowerDutyCycleOut(double power) {
    io.runPowerDutyCycleOut(power);
  }
}