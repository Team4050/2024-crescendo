// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.out.print(
        "                                                          __   __                         \n"
            + "                                                     _wr''       ''-q__                   \n"
            + "                                                  _dP                 9m_                 \n"
            + "                                                _#P                     9#_               \n"
            + "                                               d#@                       9#m              \n"
            + "                                              d##                         ###             \n"
            + "                                             J###                         ###L            \n"
            + "                                             {###K                       J###K            \n"
            + "                                             ]####K      ___aaa___      J####F            \n"
            + "                                         __gmM######_  w#P''   ''9#m  _d#####Mmw__        \n"
            + "                                      _g##############mZ_         __g##############m_     \n"
            + "                                    _d####M@PPPP@@M#######Mmp gm#########@@PPP9@M####m_   \n"
            + "                                   a###'           ,Z'#####@   ######'jg          ''M##m  \n"
            + "                                  J#@'             0L  '*##     ##@'  J#              *#K \n"
            + "                                  #'               `#    '_gmwgm_~    dF               `#_\n"
            + "                                 7F                 '#_   ]#####F   _dK                 JE\n"
            + "                                 ]                    *m__ ##### __g@'                   F\n"
            + "                                                        'PJ#####LP'                       \n"
            + "                                  `                       0######_                      ' \n"
            + "                                                        _0########_                       \n"
            + "                                      .               _d#####^#####m__              ,     \n"
            + "                                       '*w_________am#####P'   ~9#####mw_________w*'      \n"
            + "                                           ''9@#####@M''           ''P@#####@M''          \n"
            + "\n"
            + "\n"
            + "\n"
            + "\n"
            + "\n"
            + "    ███        ▄████████  ▄██████▄  ███    █▄  ▀█████████▄   ▄█          ▄████████  ▄████████  ▄█          ▄████████    ▄█████ \n"
            + "▀█████████▄   ███    ███ ███    ███ ███    ███   ███    ███ ███         ███    ███ ███    ███ ███         ███    ███   ███     \n"
            + "   ▀███▀▀██   ███    ███ ███    ███ ███    ███   ███    ███ ███         ███    █▀  ███    █▀  ███         ███    █▀    ███     \n"
            + "    ███   ▀  ▄███▄▄▄▄██▀ ███    ███ ███    ███  ▄███▄▄▄██▀  ███        ▄███▄▄▄     ███        ███        ▄███▄▄▄      ▄███▄▄▄  \n"
            + "    ███     ▀▀███▀▀▀▀▀   ███    ███ ███    ███ ▀▀███▀▀▀██▄  ███       ▀▀███▀▀▀     ███        ███       ▀▀███▀▀▀     ▀▀███▀▀▀  \n"
            + "    ███     ▀███████████ ███    ███ ███    ███   ███    ██▄ ███         ███    █▄  ███    █▄  ███         ███    █▄    ███     \n"
            + "    ███       ███    ███ ███    ███ ███    ███   ███    ███ ███▌    ▄   ███    ███ ███    ███ ███▌    ▄   ███    ███   ███     \n"
            + "   ▄████▀     ███    ███  ▀██████▀  ████████▀  ▄█████████▀  █████▄▄██   ██████████ ████████▀  █████▄▄██   ██████████   ███     \n"
            + "              ███    ███                                    ▀                                 ▀                                \n");
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.chooseControls();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
