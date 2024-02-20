package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private boolean state = false;
  private CANSparkMax intake;
  private SparkPIDController controller;
  private TalonSRX conveyor;
  //private Ultrasonic ultrasonic;
  private DigitalInput breakbeam;

  public Intake() {
    /* TODO: intake and conveyor declaration */
    intake = new CANSparkMax(Constants.MotorCANValues.intake, MotorType.kBrushless);
    breakbeam = new DigitalInput(0);
    intake.setIdleMode(IdleMode.kCoast);
    // ultrasonic = new Ultrasonic(0, 1);
    conveyor = new TalonSRX(Constants.MotorCANValues.conveyor);
    controller = intake.getPIDController();
    controller.setP(0.2);
    controller.setI(0.01);
    controller.setD(0.1);
  }

  /**
   * Activate/deactivate intake
   *
   * @param state
   */
  public void setIntake(boolean state, boolean reverse) {
    this.state = state;
    if (state) {
      //intake.set(Constants.MotorDriveValues.intakeSpeed);
      controller.setReference(-0.5, ControlType.kDutyCycle);
      return;
    } else if (reverse) {
      controller.setReference(0.5, ControlType.kDutyCycle);
      return;
    }
    controller.setReference(0, ControlType.kDutyCycle);
    //intake.set(0);
  }

  /**
   * Activate/deactivate conveyor
   *
   * @param state
   */
  public void setConveyor(double state) {
    conveyor.set(TalonSRXControlMode.PercentOutput, state);
    /*if (state) {
      conveyor.set(TalonSRXControlMode.PercentOutput, Constants.MotorDriveValues.conveyorSpeed);
      return;
    }
    conveyor.set(TalonSRXControlMode.PercentOutput, 0);*/
  }

  /**
   * Gets the ultrasonic range in millimeters
   *
   * @return
   */
  public boolean get() {
    return breakbeam.get();
  }
}
