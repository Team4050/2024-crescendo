package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class NoteStateMachine extends Command {
  private Intake intake;
  private Gun gun;
  private boolean loaded;
  private boolean cancel;

  public NoteStateMachine(Intake intake, Gun gun) {
    this.intake = intake;
    this.gun = gun;
    cancel = false;
  }

  @Override
  public void initialize() {
    loaded = intake.get();
    super.initialize();
  }

  @Override
  public void execute() {
    intake.setConveyor(-1);
    if (intake.get()) {
      gun.fire(1);
    } else {
      intake.setIntake(true, false);
    }
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntake(false, false);
    intake.setConveyor(0);
    gun.fire(0);
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    if (!loaded) {
      return intake.get();
    }
    return cancel;
  }
}
