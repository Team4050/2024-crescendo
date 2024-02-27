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
    // System.out.println("bruh real");
  }

  @Override
  public void initialize() {
    TrajectoryConfig tconfig =
        new TrajectoryConfig(Units.MetersPerSecond.of(1), Units.MetersPerSecondPerSecond.of(1));
    super.initialize();
    timer = new Timer();
    timer.start();
    ArrayList<Translation2d> list = new ArrayList<Translation2d>();
    list.add(new Translation2d(3, 0));
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            list,
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            tconfig);
    end = timer.get() + trajectory.getTotalTimeSeconds();
    // 3 means 11 inches
  }

  @Override
  public void execute() {
    drive.follow(trajectory, timer.get());
    State s = trajectory.sample(timer.get());
    System.out.println(
        "target: "
            + s.poseMeters.getX()
            + " "
            + s.poseMeters.getY()
            + " "
            + s.poseMeters.getRotation().getRadians());
    // + "\n" +
    // "speeds: " + d.leftMetersPerSecond + " " + d.rightMetersPerSecond);
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
