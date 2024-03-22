// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean logging = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryControllerPort = 1;
    public static final int kDriverFStickControllerPort = 0;
  }

  public static class RobotConstants {
    public static final double wheelRadMeters = 0.0762;
    public static final double wheelCircumMeters = wheelRadMeters * Math.PI * 2;
    public static final double gearRatio = 8.46;
    public static final double encoderToMeters = wheelCircumMeters / (42.0 * gearRatio);
    public static final double robotMassKG = 52.612179; // 103.1 lbs + 12.89 battery weight
    public static final double robotWidth = 22 * 2.54 / 100;
    public static final double robotHeight = 0.7;
    public static final double robotDepth = 30 * 2.54 / 100;
    public static final double robotTrackWidth = 0.4318;
  }

  public static class FieldConstants {
    public static Pose2d speakerLocation = new Pose2d(16.579342, 4.982717999999999, new Rotation2d(Math.toRadians(180)));
    public static Pose2d speakerLocation2 = new Pose2d(-0.038099999999999995, 5.547867999999999, new Rotation2d(0));
  }

  public static class MotorCANValues {
    // All motors are Rev NEO except for conveyor which is Talon SRX
    public static final int leftMotorFront = 2;
    public static final int leftMotorRear = 1;
    public static final int rightMotorFront = 3;
    public static final int rightMotorRear = 4;

    public static final int leftClimber = 1;
    public static final int rightClimber = 1;

    public static final int shooterWheelL = 8;
    public static final int shooterWheelR = 5;

    public static final int intake = 6;
    public static final int conveyor = 7;

    public static final int cancoderLeft = 10;
    public static final int cancoderRight = 11;
  }

  public static class MotorDriveValues {
    public static final double shooterSpeedAmp = 0.3;
    public static final double shooterSpeedSpeaker = 0.8;
    public static final double speakerTargetVelocityRPM = 0;
    public static final double ampTargetVelocityRPM = 0;
    public static final double intakeSpeed = 0.2;
    public static final double conveyorSpeed = -0.7;
  }

  public static class Sensors {
    public static final String LimelightName = "Limelight";
    public static final Transform3d RobotToCamera = new Transform3d(0, 0, 0.6477, new Rotation3d(0, Math.toRadians(35), 0));
    public static final IMUAxis robotYawAxis = IMUAxis.kRoll;
  }
}
