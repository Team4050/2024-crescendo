package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gun extends SubsystemBase {
    /* */
    private CANSparkMax shooter;

    Gun() {
        /* TODO: shooter motor declaration */
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    /**
     * Set the speed of the shooter motor
     * @param speed Output percentage -1.0 - 1.0
     */
    public void fire(double speed) {
        shooter.set(speed);
    }
}
