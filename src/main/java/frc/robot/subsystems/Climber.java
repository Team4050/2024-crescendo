package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Not yet :( */
public class Climber extends SubsystemBase {

  /* Climber can't extend again after retracting */
  public boolean extended = false;

  public Climber() {}

  public void extend() {
    if (extended) return;

    extended = true;
  }

  public void retract() {}
}
