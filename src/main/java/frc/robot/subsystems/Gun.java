package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANValues;

public class Gun extends SubsystemBase {
  /* */
  private CANSparkMax shooterL;
  private CANSparkMax shooterR;
  private ProfiledPIDController controllerL;
  private ProfiledPIDController controllerR;
  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;

  public Gun() {
    shooterL = new CANSparkMax(MotorCANValues.shooterWheelL, MotorType.kBrushless);
    shooterR = new CANSparkMax(MotorCANValues.shooterWheelR, MotorType.kBrushless);
    encoderL = shooterL.getEncoder();
    encoderR = shooterR.getEncoder();
    Constraints pidConstraints = new Constraints(1, 1);
    controllerL = new ProfiledPIDController(1, 0, 0, pidConstraints);
    controllerR = new ProfiledPIDController(1, 0, 0, pidConstraints);
  }

  @Override
  public void periodic() {
    shooterL.set(controllerL.calculate(encoderL.getVelocity()));
    shooterR.set(controllerR.calculate(encoderR.getVelocity()));
    super.periodic();
  }

  /**
   * Set the speed of the shooter motor
   *
   * @param speed Output percentage -1.0 - 1.0
   */
  public void fire(double speed, double spin) {
    controllerL.setGoal(speed + spin);
    controllerR.setGoal(speed - spin);
  }
}
