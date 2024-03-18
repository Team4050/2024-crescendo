package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANValues;

/*
 * 30% works from 22 in
 * 35%-100% work flush
 * TODO: lookup table or linear function
 */

public class Gun extends SubsystemBase {
  /* */
  private CANSparkMax shooterL;
  private CANSparkMax shooterR;
  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;

  // TODO: switch l and r
  public Gun() {
    shooterL = new CANSparkMax(MotorCANValues.shooterWheelR, MotorType.kBrushless);
    shooterR = new CANSparkMax(MotorCANValues.shooterWheelL, MotorType.kBrushless);
    shooterL.setInverted(false);
    shooterR.setInverted(true);
    shooterL.setSmartCurrentLimit(80);
    shooterR.setSmartCurrentLimit(80);
    shooterL.setIdleMode(IdleMode.kCoast);
    shooterR.setIdleMode(IdleMode.kCoast);
    shooterL.clearFaults();
    shooterR.clearFaults();
    //System.out.println(shooterL.isFollower());
    //shooterR.follow(shooterL, true);
    encoderL = shooterL.getEncoder();
    encoderR = shooterR.getEncoder();
  }

  @Override
  public void periodic() {
    /*double lpower = controllerL.calculate(encoderL.getVelocity() / 42);
    shooterL.set(-lpower);
    System.out.println("L power: " + lpower);
    shooterR.set(-controllerR.calculate(encoderR.getVelocity() / 42));
    super.periodic();*/
  }

  /**
   * Set the speed of the shooter motor
   *
   * @param speed Output percentage -1.0 - 1.0
   */
  public void fire(double speed, double spin) {
    shooterR.setVoltage((speed + spin) * 12);
    shooterL.setVoltage((speed - spin) * 12);
    System.out.println("out: " + shooterL.getAppliedOutput());
    //controllerL.setGoal(speed + spin);
    //controllerR.setGoal(speed - spin);
  }

  /**
   * Fire the shooter and compensate for distance from speaker
   * 
   * @param distance
   */
  public void fireCompensate(double distance) {
    if (distance > 1) {
      System.out.println("Distance too far to shoot");
      return;
    }
    if (distance > 0.5588) {
      fire(0.3, 0);
      return;
    }
    fire(0.35, 0);
    return;
  }
}
