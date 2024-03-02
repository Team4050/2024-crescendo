package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

/** Auto for testing autonomous movement methods */
public class TestAuto extends Command {
  private Drivetrain drive;
  private Timer timer;
  private double end;
  private Trajectory trajectory;

  public TestAuto(Drivetrain d) {
    drive = d;
  }

  @Override
  public void initialize() {
    TrajectoryConfig tconfig =
        new TrajectoryConfig(Units.MetersPerSecond.of(1), Units.MetersPerSecondPerSecond.of(1));
    super.initialize();
    timer = new Timer();
    timer.start();
    ArrayList<Pose2d> list = new ArrayList<Pose2d>();
    // 3.93954
    drive.set(0.9, 0.9);
    list.add(new Pose2d(0, 0, new Rotation2d()));
    list.add(new Pose2d(1.0, 0, new Rotation2d()));
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            list,
            tconfig);
    end = timer.get() + 0.7;
    // 3 means 11 inches
  }

  int loop = 0;

  @Override
  public void execute() {
    //drive.follow(trajectory, timer.get());
    drive.set(1.0, 1.0);
    //State s = trajectory.sample(timer.get());
    /*loop++;
    if (loop > 500) {
      System.out.println(
          "Target: "
              + s.poseMeters.getX()
              + " "
              + s.poseMeters.getY()
              + " "
              + s.poseMeters.getRotation().getDegrees());
      System.out.println(
          "Measured: "
              + drive.get().getX()
              + " "
              + drive.get().getY()
              + " "
              + drive.get().getRotation().getDegrees());
    }*/
  }

  @Override
  public void end(boolean interrupted) {
      drive.set(0, 0);
      super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > end;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    HashSet<Subsystem> set = new HashSet<Subsystem>(1);
    set.add(drive);
    return set;
  }
}
