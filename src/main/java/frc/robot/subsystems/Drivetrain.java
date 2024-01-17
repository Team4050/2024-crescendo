package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class Drivetrain extends SubsystemBase {
    /* Motors */

    PWMMotorController rightFront;
    PWMMotorController leftFront;
    PWMMotorController rightBack;
    PWMMotorController leftBack;

    private DifferentialDrive drivetrain;

    public Drivetrain() {
        leftFront.addFollower(leftBack);
        rightFront.addFollower(rightBack);
        drivetrain = new DifferentialDrive(leftFront, rightFront);
    }

    /**
     * Get the robot's speed and direction
     */
    public void get() {

    }

    public void set(double lSpeed, double rSpeed) {
        drivetrain.tankDrive(lSpeed, rSpeed);
    }

    public void set(Pose2d direction) {

    }

    /**
     * Align the robot shooter to face a point at a set distance from it
     * @param point
     * @param distance
     */
    public void align(Translation2d point, double distance) {
        
    }
}
