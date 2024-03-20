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
import frc.robot.subsystems.Intake;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

/** Auto for testing autonomous movement methods */
public class TestAuto extends Command {
  private Drivetrain drive;
  private Intake intake;
  private boolean runIntake;
  private Timer timer;
  private double end;
  private Trajectory trajectory;
  private Pose2d[] waypoints;
  private boolean reverse;
  private boolean reset;

  public TestAuto(Drivetrain d, Intake i, boolean runIntake, Pose2d[] list, boolean reverse, boolean reset) {
    drive = d;
    intake = i;
    waypoints = list;
    this.reverse = reverse;
    this.runIntake = runIntake;
    this.reset = reset;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    TrajectoryConfig tconfig =
        new TrajectoryConfig(Units.MetersPerSecond.of(1.5), Units.MetersPerSecondPerSecond.of(1));
    tconfig.setReversed(reverse);

    timer.reset();
    timer.start();

    ArrayList<Pose2d> list = new ArrayList<Pose2d>();

    if (reset) {
      drive.setPose(waypoints[0]);
    }

    System.out.println("Initial: " + drive.getTransformedIMU() + " " + drive.get());

    for (int i = 0; i < waypoints.length; i++) {
      list.add(waypoints[i]);
    }
    // 3.93954
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            list,
            tconfig);
    end = timer.get() + trajectory.getTotalTimeSeconds();
  }

  int loop = 0;

  @Override
  public void execute() {
    if (runIntake) {
      intake.setIntake(false, true);
    }
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
  public void end(boolean interrupted) {
    drive.set(0, 0);
      intake.setIntake(false, false);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    HashSet<Subsystem> set = new HashSet<Subsystem>(1);
    set.add(drive);
    set.add(intake);
    return set;
  }
}
