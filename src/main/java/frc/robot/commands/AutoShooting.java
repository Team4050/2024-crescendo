package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoShooting extends Command {
  /*int stage = 0;
  private Timer timer;
  private Drivetrain drive;
  private Intake intake;
  private Gun gun;
  private Rotation2d startingAngle;
  private Rotation2d correction = new Rotation2d(Math.toRadians(-60));
  private double end;

  AutoShooting(Drivetrain drive) {
      this.drive = drive;
  }

  @Override
  public void initialize() {
      timer = new Timer();
      timer.start();
      end = timer.get() + 2.3;
      // TODO Auto-generated method stub
      startingAngle = drive.getImu();
      super.initialize();
  }

  @Override
  public void execute() {
      if (timer.get() < 0.5) {
          intake.setConveyor(Constants.MotorDriveValues.conveyorSpeed);
          gun.fire(Constants.MotorDriveValues.shooterSpeed);
      } else if (timer.get() < 1.5) {
          drive.turn(startingAngle.rotateBy(correction));
      } else {
          drive.set(0.8, 0.8);
      }
      // TODO Auto-generated method stub
      super.execute();
  }

  @Override
  public boolean isFinished() {
      // TODO Auto-generated method stub
      return timer.get() > end;
  }*/
}
