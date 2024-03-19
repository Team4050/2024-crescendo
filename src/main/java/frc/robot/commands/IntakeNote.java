package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  private Intake intake;
  private Drivetrain drive;
  private Timer timer;

  public IntakeNote(Drivetrain drive, Intake intake) {
    this.intake = intake;
    this.drive = drive;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.start();
    super.initialize();
  }

  @Override
  public void execute() {
    drive.set(0.4, 0.4);
    intake.setIntake(true, false);
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return timer.get() > 0.5;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      HashSet<Subsystem> reqs = new HashSet<Subsystem>();
      reqs.add(drive);
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
