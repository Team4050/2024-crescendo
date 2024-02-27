package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private boolean state = false;
  private CANSparkMax intake;
  private SparkPIDController controller;
  private TalonSRX conveyor;
  // private Ultrasonic ultrasonic;
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
      controller.setReference(-0.5, ControlType.kDutyCycle);
      return;
    } else if (reverse) {
      controller.setReference(0.5, ControlType.kDutyCycle);
      return;
    }
    controller.setReference(0, ControlType.kDutyCycle);
  }

  /**
   * Activate/deactivate conveyor
   *
   * @param state
   */
  public void setConveyor(double state) {
    conveyor.set(TalonSRXControlMode.PercentOutput, state);
  }

  /**
   * Gets the beam status
   *
   * @return
   */
  public boolean get() {
    return breakbeam.get();
  }

  // TODO: Remove
  int loop = 0;

  @Override
  public void periodic() {
    /*loop++;
    if (loop > 30) {
      System.out.println("IR breakbeam:" + breakbeam.get());
    }*/
  }
}
