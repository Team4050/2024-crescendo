// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;

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
    public static final double gearRatio = 8.45;
    public static final double encoderToMeters = wheelCircumMeters / (42.0 * gearRatio);
    public static final double robotMassKG = 42.86448; //42.86448
    public static final double robotWidth = 22 * 2.54 / 100;
    public static final double robotHeight = 30 * 2.54 / 100;
    public static final double robotDepth = 30 * 2.54 / 100;
    public static final double robotTrackWidth = 0.42545;

    // TODO: find circumference of shooter wheels
    public static final double shooterEncoderToVelociy = 1 / 42;
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
    public static final Transform3d RobotToCamera = new Transform3d();
  }
}
