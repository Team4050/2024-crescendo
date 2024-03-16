package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
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
  private Pose2d[] waypoints;
  private boolean reverse;
  private boolean reset;

  public TestAuto(Drivetrain d, Pose2d[] list, boolean reverse, boolean reset) {
    drive = d;
    waypoints = list;
    this.reverse = reverse;
    this.reset = reset;
  }

  @Override
  public void initialize() {
    TrajectoryConfig tconfig =
        new TrajectoryConfig(Units.MetersPerSecond.of(2), Units.MetersPerSecondPerSecond.of(1.5));
    tconfig.setReversed(reverse);
    super.initialize();
    timer = new Timer();
    timer.start();
    ArrayList<Pose2d> list = new ArrayList<Pose2d>();
    if (reset) {
      drive.setPose(waypoints[0]);
      drive.setIMU(waypoints[0].getRotation().getDegrees());
    }
    System.out.println("Initial: " + drive.getTransformedIMU() + " " + drive.get());
    for (int i = 0; i < waypoints.length; i++) {
      list.add(waypoints[i]);
    }
    // 3.93954
    //drive.set(0.9, 0.9);
    /*list.add(new Pose2d(0, 0, new Rotation2d()));
    list.add(new Pose2d(2.0, 1.0, new Rotation2d(Math.toRadians(0))));
    list.add(new Pose2d(3.0, 0.0, new Rotation2d(Math.toRadians(-90))));
    list.add(new Pose2d(4.0, -1.0, new Rotation2d()));
    list.add(new Pose2d(5.0, 0.0, new Rotation2d(90)));
    list.add(new Pose2d(6.0, 1.0, new Rotation2d()));
    list.add(new Pose2d(7.0, 0.0, new Rotation2d()));*/
    //list.add(new Pose2d(1.4, 5.5, new Rotation2d()));
    //list.add(new Pose2d(6.0, 6.7, new Rotation2d(Math.toRadians(0))));

    //list.add(new Pose2d(6.0, 6.7, new Rotation2d(Math.toRadians(0))));
    //list.add(new Pose2d(1.4, 5.5, new Rotation2d()));
    //tconfig.setReversed(true);

    //list.add(new Pose2d(10.0, 0, new Rotation2d(Math.toRadians(0))));
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            list,
            tconfig);
    end = timer.get() + trajectory.getTotalTimeSeconds() + 4.0;
    //drive.setIMU(0);
    //drive.setPose(list.get(0));
    // 3 means 11 inches
  }

  int loop = 0;

  @Override
  public void execute() {
    drive.follow(trajectory, timer.get());
    State s = trajectory.sample(timer.get());
    loop++;
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
      System.out.println("Time: " + timer.get());
    }
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
