package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

public class AlignShooter extends Command {
  private Drivetrain drivetrain;
  // TODO: test position
  // 16.579342, 4.982717999999999 
  private static Pose2d speakerLocation = new Pose2d(16.579342, 4.982717999999999, new Rotation2d(Math.toRadians(180)));
  private static Pose2d speakerLocation2 = new Pose2d(-0.038099999999999995, 5.547867999999999, new Rotation2d(0));
  private static double distMeters = 1;
  private Pose2d targetPose;
  private Trajectory s;
  private Timer t;

  public AlignShooter(Drivetrain d) {
    drivetrain = d;
  }

  @Override
  public void initialize() {
    //ArrayList<Pose2d> points = new ArrayList<Pose2d>(0);
    //Pose2d closestSpeaker = speakerLocation;
    // Difference vector
    Translation2d dif = speakerLocation.getTranslation().minus(drivetrain.get().getTranslation());
    Translation2d dif2 = speakerLocation.getTranslation().minus(drivetrain.get().getTranslation());
    if (dif2.getNorm() < dif.getNorm()) {
      dif = dif2;
      //closestSpeaker = speakerLocation2;
    }
    targetPose = new Pose2d(drivetrain.get().getX(), drivetrain.get().getY(), new Rotation2d(dif.getX(), dif.getY()));
    // Normalize
    //dif = dif.times(1 / dif.getNorm());

    /*points.add(drivetrain.get());
    points.add(
        new Pose2d(
            // Scale difference norm by desired length
            closestSpeaker.getTranslation().plus(dif.times(distMeters)),
            // Rotation is inverse of distance norm
            new Rotation2d(-dif.getX(), -dif.getY())));
    System.out.println(drivetrain.get());
    s = TrajectoryGenerator.generateTrajectory(points, new TrajectoryConfig(1, 1));
    t = new Timer();
    t.start();
    System.out.println(t.get());
    super.initialize();*/
  }

  @Override
  public void execute() {
    //drivetrain.follow(s, t.get());
    drivetrain.control(targetPose, 0, 0);
  }

  private void alignManual() {
    Optional<EstimatedRobotPose> visionEstimation = drivetrain.getRawCameraDetection();
    if (visionEstimation.isPresent()) {
      Translation2d dif =
          speakerLocation
              .getTranslation()
              .minus(visionEstimation.get().estimatedPose.toPose2d().getTranslation());
      // Normalize
      dif = dif.times(1 / dif.getNorm());
      Rotation2d rotation = new Rotation2d(-dif.getX(), -dif.getY());
      Pose2d p =
          new Pose2d(
              // Scale difference norm by desired length
              speakerLocation.getTranslation().plus(dif.times(distMeters)),
              // Rotation is inverse of distance norm
              new Rotation2d(-dif.getX(), -dif.getY()));
      // 2 degrees tolerance
      if (Math.abs(
                  visionEstimation
                      .get()
                      .estimatedPose
                      .toPose2d()
                      .getRotation()
                      .minus(rotation)
                      .getDegrees())
              < 2.0
          && dif.getNorm() > distMeters) {
        drivetrain.set(0.2, 0.2);
      }
    }
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return t.get() > s.getTotalTimeSeconds();
  }

  @Override
  public Set<Subsystem> getRequirements() {
      HashSet<Subsystem> reqs = new HashSet<Subsystem>();
      reqs.add(drivetrain);
      return reqs;
  }

  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
  }
}
