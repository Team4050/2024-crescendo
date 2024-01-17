package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    /* If we're doing wratchets, the climber can't extend again after retracting */
    public boolean extended = false;

    Climber() {
        
    }

    public void Extend() {
        if (extended) return;

        extended = true;
    }

    public void Retract() {

    }
}
