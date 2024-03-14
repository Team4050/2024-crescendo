package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANValues;

public class Gun extends SubsystemBase {
  /* */
  private CANSparkMax shooter;
  private SparkPIDController controller;
  private ProfiledPIDController controller2;

  public Gun() {
    shooter = new CANSparkMax(MotorCANValues.shooterWheel, MotorType.kBrushless);
    controller = shooter.getPIDController();
    controller.setP(1);
    controller.setI(0);
    controller.setD(0);
    controller.setFF(0);
    controller2 = new ProfiledPIDController(1, 0, 0, new Constraints(1, 1));
  }

  @Override
  public void periodic() {
    controller2.calculate(shooter.get());
    super.periodic();
  }

  /**
   * Set the speed of the shooter motor
   *
   * @param speed Output percentage -1.0 - 1.0
   */
  public void fire(double speed) {
    controller2.setGoal(speed);
  }
}
