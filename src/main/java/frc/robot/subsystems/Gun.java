package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
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
  private double out = 0;

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
    //shooterL.follow(shooterL);
    //shooterR.follow(shooterR);
    encoderL = shooterL.getEncoder();
    encoderR = shooterR.getEncoder();
  }

  @Override
  public void periodic() {
    
  }

  /**
   * Set the speed of the shooter motor
   *
   * @param speed Output percentage -1.0 - 1.0
   */
  public void fire(double speed, double spin) {
    shooterR.setVoltage((speed + spin) * 12);
    shooterL.setVoltage((speed - spin) * 12);
  }

  public Pose2d getClosestSpeaker(Pose2d pose) {
    Translation2d dif = FieldConstants.speakerLocation.getTranslation().minus(pose.getTranslation());
    Translation2d dif2 = FieldConstants.speakerLocation2.getTranslation().minus(pose.getTranslation());
    if (dif2.getNorm() < dif.getNorm()) {
      System.out.println("Closer to blue");
      return FieldConstants.speakerLocation2;
    }
    System.out.println("Closer to red");
    return FieldConstants.speakerLocation;
  }

  /**
   * Fire the shooter and compensate for distance from speaker
   * 
   * @param distance
   */
  public void fireCompensate(Pose2d pose) {
    Pose2d speaker = getClosestSpeaker(pose);
    double distance = speaker.minus(pose).getTranslation().getNorm();
    System.out.println("Distance to speaker: " + distance);

    if (distance > 1) {
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
