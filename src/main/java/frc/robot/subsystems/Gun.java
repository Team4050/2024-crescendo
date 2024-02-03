package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gun extends SubsystemBase {
    /* */

    private boolean state = false;

    Gun() {

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public boolean state(boolean newState) {
        return state;
    }
}
