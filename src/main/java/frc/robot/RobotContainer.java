// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final RunCommand drivetrainTank;
  private final RunCommand drivetrainArcade1Stick;

  /* Controllers */
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final HazardXbox m_driverController =
      new HazardXbox(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // limelight = new PhotonCamera(Constants.Sensors.LimelightName);
    SmartDashboard.putNumber("test", 1);
    drivetrain = new Drivetrain();
    intake = new Intake();
    shooter = null;

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("test auto", new TestAuto(drivetrain));

    SmartDashboard.putData("Auto chooser", autoChooser);
    Constraints c = new Constraints(1, 0.5);
    controlProfile = new TrapezoidProfile(c);
    configureBindings();
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              drivetrain.set(Math.log(m_driverController.getLeftY() + Math.E) - 1, -(Math.log(m_driverController.getRightY() + Math.E) - 1));
            },
            drivetrain));

    drivetrainDefault = new RunCommand(
      () -> {
        double f = m_driverController.getLeftY();
        double t = -m_driverController.getRightX();
        drivetrain.set((f + t) / 1.5, (f - t) / 1.5);
      },
      drivetrain);
    drivetrainTank = new RunCommand(
      () -> {
        drivetrain.set(Math.log(m_driverController.getLeftY() + Math.E) - 1, -(Math.log(m_driverController.getRightY() + Math.E) - 1));
      },
      drivetrain);
    drivetrainArcade1Stick = new RunCommand(
      () -> {
        double f = m_driverController.getLeftY();
        double t = -m_driverController.getLeftX();
        drivetrain.set((f + t) / 1.5, (f - t) / 1.5);
      },
      drivetrain);
    
    driveControlChooser = new SendableChooser<RunCommand>();
    driveControlChooser.setDefaultOption("Default arcade control scheme", drivetrainDefault);
    driveControlChooser.addOption("Arcade control using the left stick", drivetrainArcade1Stick);
    driveControlChooser.addOption("Tank control scheme", drivetrainTank);
    SmartDashboard.putData("Drive control chooser", driveControlChooser);

    drivetrain.setDefaultCommand(drivetrainDefault);
    intake.setDefaultCommand(
        new RunCommand(
            () -> {
              intake.setIntake(m_driverController.rightTrigger().getAsBoolean(), m_driverController.b().getAsBoolean());
            },
            intake));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  public void chooseControls() {
    drivetrain.setDefaultCommand(driveControlChooser.getSelected());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command c = autoChooser.getSelected();
    autoChooser.close();
    return c; // Autos.exampleAuto(m_exampleSubsystem);
  }
}
