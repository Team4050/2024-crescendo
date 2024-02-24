// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.Drivetrain;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignShooter;
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

  //private final RunCommand drivetrainDefault;
  //private final RunCommand drivetrainTank;
  //private final RunCommand drivetrainArcade1Stick;

  /* Controllers */
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final HazardXbox m_driverController =
      new HazardXbox(OperatorConstants.kDriverControllerPort);

  private final Joystick m_driverFStick = new Joystick(OperatorConstants.kDriverControllerPort);
  //private final Joystick jRight = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // limelight = new PhotonCamera(Constants.Sensors.LimelightName);
    SmartDashboard.putNumber("test", 1);
    drivetrain = new Drivetrain(new Pose2d(0, 0, new Rotation2d()));
    intake = new Intake();
    shooter = null;

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("test auto", new TestAuto(drivetrain));

    SmartDashboard.putData("Auto chooser", autoChooser);
    Constraints c = new Constraints(1, 0.5);
    controlProfile = new TrapezoidProfile(c);
    configureBindings();
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      double f = -m_driverFStick.getY();
      double t = m_driverFStick.getX();
      drivetrain.set((f + t), (f - t));
    }, drivetrain));
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
    //drivetrain.setDefaultCommand(driveControlChooser.getSelected());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command c;// = autoChooser.getSelected();
    //autoChooser.close();
    c = new TestAuto(drivetrain);
    return c; // Autos.exampleAuto(m_exampleSubsystem);
  }
}
