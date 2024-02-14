package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private boolean state = false;
  private CANSparkMax intake;
  private SparkPIDController controller;
  private TalonSRX conveyor;
  private Ultrasonic ultrasonic;

  public Intake() {
    /* TODO: intake and conveyor declaration */
    intake = new CANSparkMax(Constants.MotorCANValues.intake, MotorType.kBrushless);
    // ultrasonic = new Ultrasonic(0, 1);
    conveyor = null;
    controller = intake.getPIDController();
    controller.setP(0.2);
    controller.setI(0);
    controller.setD(0);
  }

  /**
   * Activate/deactivate intake
   *
   * @param state
   */
  public void setIntake(boolean state) {
    this.state = state;
    if (state) {
      //intake.set(Constants.MotorDriveValues.intakeSpeed);
      controller.setReference(2048, ControlType.kVelocity);
      return;
    }
    controller.setReference(0, ControlType.kVelocity);
    //intake.set(0);
  }

  /**
   * Activate/deactivate conveyor
   *
   * @param state
   */
  public void setConveyor(boolean state) {
    if (state) {
      conveyor.set(TalonSRXControlMode.PercentOutput, Constants.MotorDriveValues.conveyorSpeed);
      return;
    }
    conveyor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * Gets the ultrasonic range in millimeters
   *
   * @return
   */
  public double get() {
    return ultrasonic.getRangeMM();
  }
}
