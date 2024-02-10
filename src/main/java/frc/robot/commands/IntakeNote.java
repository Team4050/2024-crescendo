package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  private Gun gun;
  private Intake intake;

  @Override
  public void initialize() {
    gun = new Gun();
    intake = new Intake();
    super.initialize();
  }

  @Override
  public void execute() {
    intake.setIntake(true);
    intake.setConveyor(true);

    // TODO: better logic & determine target distance
    while (intake.get() > 40) {}

    intake.setIntake(false);
    intake.setConveyor(false);
    super.execute();
  }
}
