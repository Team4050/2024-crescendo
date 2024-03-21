// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.PathweaverTest;
import frc.robot.commands.TestAuto;
import frc.robot.commands.TimeTesting;
import frc.robot.commands.Profiling;
import frc.robot.commands.Shoot;
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
  private final PowerDistribution pdh;

  /* Commands */

  /* Sensors */

  /* Misc */
  private SendableChooser<Command> autoChooser;
  private SendableChooser<RunCommand> driveControlChooser;
  private TrapezoidProfile controlProfile;

  private final RunCommand drivetrainDefault;
  private final RunCommand drivetrainRamp;
  private final RunCommand drivetrainRampDefensive;
  private final RunCommand drivetrainScaledCurve;
  private final RunCommand drivetrainTank;
  private final RunCommand drivetrainArcade1Stick;
  private boolean reverse = false;
  private AlignShooter alignShooter;

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
  private final Trigger leftTrigger = m_secondaryController.leftTrigger();
  private final Trigger rightTrigger = m_secondaryController.rightTrigger();
  private final Trigger povUp = m_secondaryController.povUp();
  private final Trigger povDown = m_secondaryController.povDown();
  private final Trigger povLeft = m_secondaryController.povLeft();
  private final Trigger povRight = m_secondaryController.povRight();
  //private final Joystick m_driverJoystick =
  //    new Joystick(OperatorConstants.kDriverFStickControllerPort);

  private double adjustedShooterSpeed = 0.6;
  private double shooterspin = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings
    // limelight = new PhotonCamera(Constants.Sensors.LimelightName);
    pdh = new PowerDistribution(1, ModuleType.kRev);
    drivetrain = new Drivetrain(new Pose2d(0, 0, new Rotation2d(0)));
    //drivetrain.setPose(new Pose2d(1.4, 6.6, new Rotation2d(-180)));
    //drivetrain.setIMU(-180);
    intake = new Intake();
    shooter = new Gun();

    // SmartDashboard.putData("Shooter speed", 1);
    SmartDashboard.putNumber("shooter", Constants.MotorDriveValues.shooterSpeedAmp);
    SmartDashboard.putNumber("shooter spin", Constants.MotorDriveValues.shooterSpeedAmp);

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Default auto: time sequence", new SequentialCommandGroup(
      new Shoot(shooter, intake),
      new TestAuto(drivetrain, intake, true, new Pose2d[] {new Pose2d(1.4, 5.5, new Rotation2d(Math.toRadians(0))), new Pose2d(4.3, 5.5, new Rotation2d(Math.toRadians(0)))}, false, true),
      //new TestAuto(drivetrain, intake, true, new Pose2d[] {new Pose2d(5, 6.6, new Rotation2d(Math.toRadians(0))), new Pose2d(5.5, 6.6, new Rotation2d(Math.toRadians(0)))}, false, false),
      new TestAuto(drivetrain, intake, false, new Pose2d[] {new Pose2d(4.3, 5.5, new Rotation2d(Math.toRadians(0))), new Pose2d(1.4, 5.5, new Rotation2d(Math.toRadians(0)))}, true, false),
      new Shoot(shooter, intake),
      new Shoot(shooter, intake),
      new TestAuto(drivetrain, intake, true, new Pose2d[] {new Pose2d(1.4, 5.5, new Rotation2d(Math.toRadians(0))), new Pose2d(6, 6, new Rotation2d(Math.toRadians(0)))}, false, false)
    ));
    //autoChooser.addOption("Test auto 2", new TestAuto(drivetrain, intake, true, new Pose2d[] {new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0))}, false, true));
    //autoChooser.addOption("nothing", new RunCommand(() -> {}, drivetrain));
    //autoChooser.addOption("Align", new AlignShooter(drivetrain, new Pose2d(1.4, 5.5, new Rotation2d())));
    autoChooser.addOption("Time sequence", 
        new SequentialCommandGroup(
            new Shoot(shooter, intake),
            new TimeTesting(drivetrain, intake, 0.75, false, true),
            //new IntakeNote(drivetrain, intake),
            new TimeTesting(drivetrain, intake, -0.75, false, false),
            new Shoot(shooter, intake),
            new Shoot(shooter, intake),
            new TimeTesting(drivetrain, intake, 0.75, false, false)));
    autoChooser.addOption("Steal from middle",
        new SequentialCommandGroup(
            new Shoot(shooter, intake),
            new TimeTesting(drivetrain, intake, 0.75, false, true),
            //new IntakeNote(drivetrain, intake),
            new TimeTesting(drivetrain, intake, -0.75, false, false),
            new Shoot(shooter, intake),
            new Shoot(shooter, intake),
            new TimeTesting(drivetrain, intake, 0.75, false, false),
            new TimeTesting(drivetrain, intake, 45, true, false),
            new TimeTesting(drivetrain, intake, 1.6, false, true)));
    autoChooser.addOption("Turning", new TimeTesting(drivetrain, intake, 120, true, false));

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
              double t = (Math.log(m_driverController.getRightX(0.04f) + Math.E) - 1) / 1.3;
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
                t = (Math.log(m_driverController.getRightX(0.04f) + Math.E) - 1) / 1.3;
              }
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t), (f - t));
            },
            drivetrain);
    /*drivetrainFStickDefault =
        new RunCommand(
            () -> {
              double f = -m_driverJoystick.getY();
              double t = m_driverJoystick.getZ();
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t) / 1.5, (f - t) / 1.5);
            },
            drivetrain);*/
    drivetrainScaledCurve = 
        new RunCommand(
          () -> {
              double f = -m_driverController.getLeftY();
              double t = 0;
              if (mb.getAsBoolean()) {
                t = m_driverController.getRightX(0.04f);
              } else {
                t = (Math.tan(m_driverController.getRightX(0.04f) * 1.25)) * (1 - Math.pow(f * 0.707, 2)) / 3;
              }
              //System.out.println(t);
              /*if (Math.abs(m_driverController.getRightX(0.04f)) > 0.8 && f < 0.2) {
                t = m_driverController.getRightX(0.04f);
              }*/
              if (reverse) {
                f = -f;
              }
              drivetrain.set((f + t), (f - t));
        }, drivetrain);
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
              drivetrain.set((f + t) / 1.5, (f - t) / 1.5);
            },
            drivetrain);

    driveControlChooser = new SendableChooser<RunCommand>();
    driveControlChooser.setDefaultOption("Default arcade control scheme (scaled curve)", drivetrainScaledCurve);
    driveControlChooser.addOption("Drivetrain arcade with turn ramping control", drivetrainRamp);
    driveControlChooser.addOption(
        "Drivetrain arcade with alt turning (primary B button)", drivetrainRampDefensive);
    driveControlChooser.addOption("Drivetrain arcade with turn ramping scaled by forward speed", drivetrainScaledCurve);
    //driveControlChooser.addOption(
    //    "Arcade control using a flight stick for primary control", drivetrainArcade1Stick);
    //driveControlChooser.addOption("Arcade control using the left stick", drivetrainArcade1Stick);
    //driveControlChooser.addOption("Tank control scheme", drivetrainTank);
    SmartDashboard.putData("Drive control chooser", driveControlChooser);

    drivetrain.setDefaultCommand(drivetrainDefault);
    intake.setDefaultCommand(
        new RunCommand(
            () -> {
              intake.setIntake(rightTrigger.getAsBoolean(), b.getAsBoolean());

              if (x.getAsBoolean()) {
                intake.setConveyor(Constants.MotorDriveValues.conveyorSpeed);
              } else {
                intake.setConveyor(0);
              }
            },
            intake));
    shooter.setDefaultCommand(
        new RunCommand(
            () -> {
              if (y.getAsBoolean()) {
                shooter.fire(0.35, 0);
              } else if (leftTrigger.getAsBoolean()) {
                shooter.fire(0.6, 0);
              } else if (povUp.getAsBoolean()) {
                shooter.fire(0.1, 0);
              } else if (povDown.getAsBoolean()) {
                shooter.fire(0.14, 0);
              } else if (povLeft.getAsBoolean()) {
                shooter.fire(0.12, 0);
              } else if (povRight.getAsBoolean()) {
                shooter.fire(0.16, 0);
                // A B start
              } else if (x.getAsBoolean()) {
                shooter.fire(0.35, 0);
              } else {
                shooter.fire(0, 0);
              }

            },
            shooter));
  }

  public void chooseControls() {
    drivetrain.setDefaultCommand(driveControlChooser.getSelected());
    adjustedShooterSpeed = SmartDashboard.getNumber("shooter", 0);
    shooterspin = SmartDashboard.getNumber("shooter spin", 0);
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

  public void periodic() {
    if (pdh.getVoltage() < 12.0) {
      pdh.setSwitchableChannel(false);
    }
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
