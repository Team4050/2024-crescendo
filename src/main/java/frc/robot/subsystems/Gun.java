package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANValues;

public class Gun extends SubsystemBase {
  /* */
  private CANSparkMax shooter;
  private SparkPIDController pid;

  public Gun() {
    shooter = new CANSparkMax(MotorCANValues.shooterWheel, MotorType.kBrushless);
    pid = shooter.getPIDController();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  /**
   * Set the speed of the shooter motor
   *
   * @param speed Output percentage -1.0 - 1.0
   */
  public void fire(double speed) {
    shooter.set(speed);
    // shooter.setVoltage(12);
  }
}
