package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleOnce extends SequentialCommandGroup {
    
  /** Creates a new ControllerRumbleOnce. */
  public ControllerRumbleOnce(double duration, double intensity, CommandXboxController controller, RumbleType type) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->controller.setRumble(type, intensity)),
      new WaitCommand(duration),
      new InstantCommand(()->controller.setRumble(type, 0))
    );
  }
    
}
