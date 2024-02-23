package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;

public class AlignShooter extends Command {
  private Drivetrain drivetrain;
  // TODO: not have speaker position be 0, 0
  private static Pose2d speakerLocation = new Pose2d(0, 0, new Rotation2d());
  private static double distMeters = 1;
  private Trajectory s;
  private Timer t;

  public AlignShooter(Drivetrain d) {
    drivetrain = d;
  }

  @Override
  public void initialize() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>(0);
    // Difference vector
    Translation2d dif = speakerLocation.getTranslation().minus(drivetrain.get().getTranslation());
    // Normalize
    dif = dif.times(1 / dif.getNorm());
    points.add(drivetrain.get());
    points.add(
        new Pose2d(
            // Scale difference norm by desired length
            speakerLocation.getTranslation().plus(dif.times(distMeters)),
            // Rotation is inverse of distance norm
            new Rotation2d(-dif.getX(), -dif.getY())));
    System.out.println(drivetrain.get());
    s = TrajectoryGenerator.generateTrajectory(points, new TrajectoryConfig(0.5, 0.1));
    t = new Timer();
    t.start();
    System.out.println(t.get());
    super.initialize();
  }

  @Override
  public void execute() {
    drivetrain.follow(s, t.get());
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return t.get() > s.getTotalTimeSeconds();
  }
}
