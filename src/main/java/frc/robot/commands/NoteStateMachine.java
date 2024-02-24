package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class NoteStateMachine extends Command {
  private Intake intake;
  private Gun gun;
  private Timer timer;
  private boolean loaded;

  public NoteStateMachine(Intake intake, Gun gun) {
    this.intake = intake;
    this.gun = gun;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    loaded = intake.get();
    timer = new Timer();
    cooldown = timer.get();
    super.initialize();
  }

  private double cooldown = 0;
  private boolean triggered = false;
  @Override
  public void execute() {
    if (intake.get() && triggered && (timer.get() > cooldown)) {
      pass++;
      triggered = true;
      cooldown = timer.get() + 0.5;
    } else {
      triggered = false;
    }

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

  int pass = 0;
  @Override
  public boolean isFinished() {
    if (!loaded) {
      return pass == 2;
    }
    return false;
  }
}
