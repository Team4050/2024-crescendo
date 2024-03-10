// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TestAuto;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Subsystems */
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Gun shooter;

  /* Commands */

  /* Sensors */

  /* Misc */
  private SendableChooser<Command> autoChooser;
  private SendableChooser<RunCommand> driveControlChooser;
  private TrapezoidProfile controlProfile;

  private final RunCommand drivetrainDefault;
  private final RunCommand drivetrainRamp;
  private final RunCommand drivetrainRampDefensive;
  private final RunCommand drivetrainFStickDefault;
  private final RunCommand drivetrainTank;
  private final RunCommand drivetrainArcade1Stick;
  private boolean reverse = false;

  /* Controllers */
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final HazardXbox m_driverController =
      new HazardXbox(OperatorConstants.kDriverControllerPort);
  private final HazardXbox m_secondaryController =
      new HazardXbox(OperatorConstants.kSecondaryControllerPort);
  private final Trigger x = m_secondaryController.x();
  private final Trigger y = m_secondaryController.y();
  private final Trigger b = m_secondaryController.b();
  private final Trigger mb = m_driverController.b();
  private final Trigger rightTrigger = m_secondaryController.rightTrigger();
  private final Joystick m_driverJoystick =
      new Joystick(OperatorConstants.kDriverFStickControllerPort);

  private double adjustedShooterSpeed = 0.6;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // limelight = new PhotonCamera(Constants.Sensors.LimelightName);
    drivetrain = new Drivetrain(new Pose2d(0, 0, new Rotation2d()));
    intake = new Intake();
    shooter = new Gun();

    // SmartDashboard.putData("Shooter speed", 1);
    SmartDashboard.putNumber("shooter", -Constants.MotorDriveValues.shooterSpeed);

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Default auto: leave area", new TestAuto(drivetrain));
    autoChooser.addOption("nothing", new RunCommand(() -> {}, drivetrain));

    SmartDashboard.putData("Auto chooser", autoChooser);

    drivetrainDefault =
        new RunCommand(
            () -> {
              double f = -m_driverController.getLeftY();
              double t = m_driverController.getRightX(0.04f) / 1.8;
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t), (f - t));
            },
            drivetrain);
    drivetrainRamp =
        new RunCommand(
            () -> {
              double f = -m_driverController.getLeftY();
              double t = (Math.log(m_driverController.getRightX(0.04f) + Math.E) - 1) / 1.7;
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t), (f - t));
            },
            drivetrain);
    drivetrainRampDefensive =
        new RunCommand(
            () -> {
              double f = -m_driverController.getLeftY();
              double t = 0;
              if (mb.getAsBoolean()) {
                t = m_driverController.getRightX();
              } else {
                t = (Math.log(m_driverController.getRightX(0.04f) + Math.E) - 1) / 1.7;
              }
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t), (f - t));
            },
            drivetrain);
    drivetrainFStickDefault =
        new RunCommand(
            () -> {
              double f = -m_driverJoystick.getY();
              double t = m_driverJoystick.getZ();
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t) / 1.5, (f - t) / 1.5);
            },
            drivetrain);
    drivetrainTank =
        new RunCommand(
            () -> {
              if (reverse) {
                drivetrain.set(
                    Math.log(-m_driverController.getLeftY() + Math.E) - 1,
                    Math.log(m_driverController.getRightY() + Math.E) - 1);
              } else {
                drivetrain.set(
                    Math.log(m_driverController.getLeftY() + Math.E) - 1,
                    Math.log(-m_driverController.getRightY() + Math.E) - 1);
              }
            },
            drivetrain);
    drivetrainArcade1Stick =
        new RunCommand(
            () -> {
              double f = -m_driverController.getLeftY();
              double t = m_driverController.getLeftX(0.04f);
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t) / 1.5, (f - t) / 1.5);
            },
            drivetrain);
    /*m_driverController.b().onTrue(new RunCommand(() -> {
      reverse = !reverse;
    }, (Subsystem[])null));*/

    driveControlChooser = new SendableChooser<RunCommand>();
    driveControlChooser.setDefaultOption("Default arcade control scheme", drivetrainDefault);
    driveControlChooser.addOption("Drivetrain arcade with turn ramping control", drivetrainRamp);
    driveControlChooser.addOption(
        "Drivetrain arcade with alt turning (primary B button)", drivetrainRampDefensive);
    driveControlChooser.addOption(
        "Arcade control using a flight stick for primary control", drivetrainArcade1Stick);
    driveControlChooser.addOption("Arcade control using the left stick", drivetrainArcade1Stick);
    driveControlChooser.addOption("Tank control scheme", drivetrainTank);
    SmartDashboard.putData("Drive control chooser", driveControlChooser);

    drivetrain.setDefaultCommand(drivetrainDefault);
    intake.setDefaultCommand(
        new RunCommand(
            () -> {
              intake.setIntake(rightTrigger.getAsBoolean(), b.getAsBoolean());

              // Robot error on X press with conveyor enabled
              if (x.getAsBoolean() || y.getAsBoolean()) {
                intake.setConveyor(Constants.MotorDriveValues.conveyorSpeed);
              } else {
                intake.setConveyor(0);
              }
            },
            intake));
    shooter.setDefaultCommand(
        new RunCommand(
            () -> {
              if (x.getAsBoolean()) {
                shooter.fire(Constants.MotorDriveValues.shooterSpeed);
              } else if (y.getAsBoolean()) {
                shooter.fire(-adjustedShooterSpeed);
              } else {
                shooter.fire(0);
              }
            },
            shooter));
  }

  public void chooseControls() {
    drivetrain.setDefaultCommand(driveControlChooser.getSelected());
    adjustedShooterSpeed = SmartDashboard.getNumber("shooter", 0);
    /*switch (driveControlChooser.getSelected()) {
      case "def":
        drivetrain.setDefaultCommand(drivetrainDefault);
        break;
      case "defRamp":
        drivetrain.setDefaultCommand(drivetrainRamp);
        break;
      case "fstick":
        drivetrain.setDefaultCommand(drivetrainFStickDefault);
        break;
      case "arcade1stick":
        drivetrain.setDefaultCommand(drivetrainArcade1Stick);
        break;
      case "tank":
        drivetrain.setDefaultCommand(drivetrainTank);
        break;
      default:
        drivetrain.setDefaultCommand(drivetrainDefault);
        break;
    }*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command c = autoChooser.getSelected();
    // TODO: Use this code if fetching from chooser fails
    /*switch (c) {
      case "defaultAuto":
        return new TestAuto(drivetrain);
        break;
      case "nothing":
        return new RunCommand(() -> {}, drivetrain);
        break;
      default:
        return new RunCommand(() -> {}, drivetrain);
        break;
    }*/
    // TODO: close resource since we're not using it twice in one run?
    // autoChooser.close();
    // c = new TestAuto(drivetrain);
    return c;
  }
}
