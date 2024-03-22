package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private boolean state = false;
  private CANSparkMax intake;
  private SparkPIDController controller;
  private TalonSRX conveyor;
  private DigitalInput breakbeam;
  // TODO: Reduce backup time, 0.2 is rather long
  private double backTime = 0.2;
  private double store = 0;
  private Timer timer;

  public Intake() {
    intake = new CANSparkMax(Constants.MotorCANValues.intake, MotorType.kBrushless);
    breakbeam = new DigitalInput(0);
    intake.setIdleMode(IdleMode.kCoast);
    conveyor = new TalonSRX(Constants.MotorCANValues.conveyor);
    controller = intake.getPIDController();
    controller.setP(0.2);
    controller.setI(0.01);
    controller.setD(0.1);
    timer = new Timer();
    timer.start();
  }

  /**
   * Activate/deactivate intake
   *
   * @param state
   */
  public void setIntake(boolean state, boolean reverse) {
    this.state = state;
    if (state) {
      controller.setReference(-1.0, ControlType.kDutyCycle);
      return;
    } else if (reverse) {
      controller.setReference(1.0, ControlType.kDutyCycle);
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

  public void setConveyorWithBackoff(boolean state) {
    if (state) {
      conveyor.set(TalonSRXControlMode.PercentOutput, Constants.MotorDriveValues.conveyorSpeed);
      store = timer.get() + backTime;
    } else if (timer.get() < store) {
      conveyor.set(TalonSRXControlMode.PercentOutput, -Constants.MotorDriveValues.conveyorSpeed);
    } else {
      conveyor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  /**
   * Gets the beam status
   *
   * @return
   */
  public boolean get() {
    return breakbeam.get();
  }
}
