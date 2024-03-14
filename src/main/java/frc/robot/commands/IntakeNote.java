package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  private Gun gun;
  private Intake intake;
  private Timer timer;
  private double conveyorSpeed = -1.0;

  public IntakeNote(Gun gun, Intake intake) {
    this.intake = intake;
    this.gun = gun;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    gun = new Gun();
    intake = new Intake();
    timer.start();
    super.initialize();
  }

  @Override
  public void execute() {
    intake.setIntake(true, false);
    intake.setConveyor(conveyorSpeed);

    // TODO: better logic & determine target distance

    super.execute();
  }

  @Override
  public boolean isFinished() {
      return intake.get() || timer.get() > 4.0;
  }

  @Override
  public void end(boolean interrupted) {
      intake.setIntake(false, false);
      intake.setConveyor(0);
      super.end(interrupted);
  }
}
