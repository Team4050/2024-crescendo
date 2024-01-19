package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Drivetrain extends SubsystemBase {
    /* Motors */

    TalonSRX rightFront;
    TalonSRX leftFront;
    TalonSRX rightBack;
    TalonSRX leftBack;

    private DifferentialDriveKinematics kinematics;

    public Drivetrain() {
        leftFront = new TalonSRX(Constants.MotorCANValues.leftMotorFront);
        rightFront = new TalonSRX(Constants.MotorCANValues.rightMotorFront);
        leftBack = new TalonSRX(Constants.MotorCANValues.leftMotorRear);
        rightBack = new TalonSRX(Constants.MotorCANValues.rightMotorRear);
        kinematics = new DifferentialDriveKinematics(0.2);
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
}
