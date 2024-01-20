package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import org.opencv.core.RotatedRect;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Drivetrain extends SubsystemBase {
    /* Motors */

    TalonSRX rightFront;
    TalonSRX leftFront;
    TalonSRX rightBack;
    TalonSRX leftBack;

    private PIDController velocity;
    private PIDController angle;

    private DifferentialDrive drivetrain;
    private Pose2d position;
    private PoseEstimator estimator;

    private DifferentialDriveKinematics kinematics;

    public Drivetrain(Pose2d initial) {
        leftFront = new TalonSRX(Constants.MotorCANValues.leftMotorFront);
        rightFront = new TalonSRX(Constants.MotorCANValues.rightMotorFront);
        leftBack = new TalonSRX(Constants.MotorCANValues.leftMotorRear);
        rightBack = new TalonSRX(Constants.MotorCANValues.rightMotorRear);
        kinematics = new DifferentialDriveKinematics(0.2);
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

    /**
     * Speed in m/s
     * @param lSpeed
     * @param rSpeed
     */
    public void set(double lSpeed, double rSpeed) {
        leftFront.set(TalonSRXControlMode.PercentOutput, lSpeed);
        leftBack.set(TalonSRXControlMode.PercentOutput, lSpeed);
        rightFront.set(TalonSRXControlMode.PercentOutput, rSpeed);
        rightBack.set(TalonSRXControlMode.PercentOutput, rSpeed);
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
