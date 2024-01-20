package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class Drivetrain extends SubsystemBase {
    /* Motors */

    /* CIM then NEO */
    private PWMMotorController right;
    private PWMMotorController left;

    private PIDController velocity;
    private PIDController angle;

    private DifferentialDrive drivetrain;
    private Pose2d position;
    private PoseEstimator estimator;

    public Drivetrain(Pose2d initial) {
        drivetrain = new DifferentialDrive(left, right);
        position = initial;
        //TODO: tune
        velocity = new PIDController(0, 0, 0);
        angle = new PIDController(0, 0, 0);
    }

    public Drivetrain() {
        this(new Pose2d());
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

    /**
     * Tells the robot to go to a point on the field
     * @param target
     */
    public void goTo(Pose2d target) {
        Transform2d v = target.minus(estimator.getEstimatedPosition());
        turnTo(v.getRotation());
        /*
         * Term for velocity
         * Term for ang. velocity
         * 
         */

        velocity.calculate(Math.sqrt(Math.pow(v.getX(), 2) + Math.pow(v.getY(), 2)));
    }

    public void turnTo(Rotation2d rotation) {
        angle.calculate(rotation.minus(estimator.getEstimatedPosition().getRotation()).getRadians());
    }
}
