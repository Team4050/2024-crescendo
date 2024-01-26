package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.hazard.HazardVision;

import java.util.Optional;

import org.opencv.core.RotatedRect;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N5;
//import edu.wpi.first.math.proto.System;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
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
    private LTVDifferentialDriveController controller;

    private DifferentialDrive drivetrain;
    private Pose2d position;
    private DifferentialDrivePoseEstimator estimator;
    private HazardVision vision;

    private DifferentialDriveKinematics kinematics;

    public Drivetrain(Pose2d initial) {
        leftFront = new TalonSRX(Constants.MotorCANValues.leftMotorFront);
        rightFront = new TalonSRX(Constants.MotorCANValues.rightMotorFront);
        leftBack = new TalonSRX(Constants.MotorCANValues.leftMotorRear);
        rightBack = new TalonSRX(Constants.MotorCANValues.rightMotorRear);
        kinematics = new DifferentialDriveKinematics(0.2);
        estimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(0), 0, 0, initial);
        position = initial;
        //TODO: tune
        velocity = new PIDController(0, 0, 0);
        angle = new PIDController(0, 0, 0);
        vision = new HazardVision();
        Vector<N5> qelems = VecBuilder.fill(0.2, 0.2, 0.2, 0.2, 0.2);
        Vector<N2> relems = VecBuilder.fill(0.2, 0.2);

        /*
         * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#Hfootnote.27
         * State vector (velocities)
         * x = [ vl ]
         *     [ vr ]
         * 
         * Input vec (Voltages)
         * u = [ Vl ]
         *     [ Vr ]
         * 
         * r = wheel radius  rb = wheelbase radius  G = gear ratio between wheels and motor (left/right)
         * J = moment of inertia  m = robot mass
         * Kt = input gain, Kv = system gain
         * R = motor resistance (left/right)
         * torque = G * Kt / R * V - G^2 * Kt / Kv * R * w
         * C1 = -Gl^2 * Kt / Kv * R * r^2
         * C2 = Gl * Kt / R * r
         * C3 = -Gr^2 * Kt / Kv * R * r^2
         * C4 = Gr * Kt / R * r
         * c1 = 1 / m + rb^2 / J
         * c2 = 1 / m - rb^2 / J
         * 
         * Input matrix
         * A = [ c1 * C1  c2 * C3 ]
         *     [ c2 * C1  c1 * C3 ]
         * 
         * System matrix
         * B = [ c1 * C2  c2 * C4 ]
         *     [ c2 * C2  c1 * C4 ]
         */
        Matrix<N2, N2> A = MatBuilder.fill(N2.instance, N2.instance, -1, 0, 0, -1);
        Matrix<N2, N2> B = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);
        Matrix<N2, N2> C = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);
        Matrix<N2, N2> D = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);

        double maxV =
        A.solve(B.times(VecBuilder.fill(12.0, 12.0))).times(-1.0).get(0, 0);
        System.out.println(maxV);

        //track width ~20 inch?
        controller = new LTVDifferentialDriveController(new LinearSystem<N2, N2, N2>(A, B, C, D), 0.508, qelems, relems, 2);
    }

    public Drivetrain() {
        this(new Pose2d());
    }

    /**
     * Get the robot's speed and direction
     */
    public Pose2d get() {
        Optional<EstimatedRobotPose> o = vision.getEstimatedGlobalPose(position);
        if (!o.isEmpty()) {
            position = o.get().estimatedPose.toPose2d();
        }
        return position;
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
        //angle.calculate(rotation.minus(estimator.getEstimatedPosition().getRotation()).getRadians());
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        Optional<EstimatedRobotPose> pose = vision.getEstimatedGlobalPose(position);
        //System.out.println("periodic");
        if (pose.isPresent()) {
            System.out.println("Vision estimated pose:" + pose.get().estimatedPose.toString());
            position = pose.get().estimatedPose.toPose2d();
        }
    }

    public DifferentialDriveWheelVoltages follow(Trajectory s, double t) {
        State p = s.sample(t);
        //add velocity references
        return controller.calculate(estimator.getEstimatedPosition(), leftFront.getSelectedSensorVelocity(), rightFront.getSelectedSensorVelocity(), p);
    }
}
