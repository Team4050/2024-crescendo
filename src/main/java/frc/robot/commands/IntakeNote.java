package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  private Gun gun;
  private Intake intake;
  private Timer timer;

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
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return timer.get() > 4.0;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      HashSet<Subsystem> reqs = new HashSet<Subsystem>();
      reqs.add(gun);
      reqs.add(intake);
      return reqs;
  }

  @Override
  public void end(boolean interrupted) {
      intake.setIntake(false, false);
      intake.setConveyor(0);
      super.end(interrupted);
  }
}
