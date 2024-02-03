package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private boolean state = false;
    private CANSparkMax intake;
    private TalonSRX conveyor;

    Intake() {

    }

    public void set() {
        
    }

    public void setConveyor() {

    }
}
