package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

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
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N5;
//import edu.wpi.first.math.proto.System;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
    /* Motors */

    private PWMMotorController left;
    private PWMMotorController right;

    private double wheelRadMeters = 0.1524;
    private double wheelCircumMeters = wheelRadMeters * Math.PI * 2;

    private LTVDifferentialDriveController controller;
    private RamseteController ramController;

    private DifferentialDrive drivetrain;
    private Pose2d position;
    private DifferentialDrivePoseEstimator estimator;
    private HazardVision vision;

    //1.15 degree drift over 2 and a half minutes
    private ADIS16470_IMU imu;

    private DifferentialDriveKinematics kinematics;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    private PIDController leftControl;
    private PIDController rightControl;

    public Drivetrain(Pose2d initial) {
        left = new PWMSparkMax(Constants.MotorCANValues.leftMotorFront);
        left.addFollower(new PWMSparkMax(Constants.MotorCANValues.leftMotorRear));
        right = new PWMSparkMax(Constants.MotorCANValues.rightMotorFront);
        right.addFollower(new PWMSparkMax(Constants.MotorCANValues.rightMotorRear));

        leftControl = new PIDController(kP, kI, kD);
        rightControl = new PIDController(kP, kI, kD);

        kinematics = new DifferentialDriveKinematics(0.508);
        estimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(0), 0, 0, initial);
        position = initial;
        vision = new HazardVision();
        imu = new ADIS16470_IMU();
        imu.configCalTime(ADIS16470_IMU.CalibrationTime._128ms);
        imu.calibrate();
        Vector<N5> qelems = VecBuilder.fill(0.2, 0.2, 0.2, 0.2, 0.2);
        Vector<N2> relems = VecBuilder.fill(0.01, 0.01);

        Matrix<N2, N2> A = MatBuilder.fill(N2.instance, N2.instance, -0.2, 0.1, 0.1, -0.2);
        Matrix<N2, N2> B = MatBuilder.fill(N2.instance, N2.instance, 0.2, -0.1, -0.1, 0.2);
        Matrix<N2, N2> C = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);
        Matrix<N2, N2> D = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);

        double maxV =
        A.solve(B.times(VecBuilder.fill(12.0, 12.0))).times(-1.0).get(0, 0);
        System.out.println(maxV);

        //Gear ratio for 2022 was 10.75:1
        LinearSystem<N2, N2, N2> plant = buildPlant(0.0762, 0.27305, 0.118, 0.118, 5.0, 40.0, 1, 1, 1, 1);

        //track width ~20 inch?
        controller = new LTVDifferentialDriveController(plant, 0.508, qelems, relems, 0.02);

        ramController = new RamseteController(0.2, 0.5);
    }

    private void configMotor(TalonSRX m) {
        m.config_kP(0, 0);
        m.config_kI(0, 0);
        m.config_kD(0, 0);
        m.config_kF(0, 0);
    }

    private LinearSystem<N2, N2, N2> buildPlant(double r, double rb, double Gl, double Gr, double J, double m, double Kt, double Kv, double Rl, double Rr) {
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
         * J = moment of inertia  m = robot mass TODO:figure out mass of Omega & current robot
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
        double C1 = -Math.pow(Gl, 2) * Kt / (Rl * Kv * Math.pow(r, 2));
        double C2 = Gl * Kt / (Rl * r);
        double C3 = -Math.pow(Gr, 2) * Kt / (Rr * Kv * Math.pow(r, 2));
        double C4 = Gr * Kt / (Rr * r);
        double c1 = 1 / m + (Math.pow(rb, 2) / J);
        double c2 = 1 / m - (Math.pow(rb, 2) / J);

        System.out.println("constants: " + c1 + " " + c2);

        Matrix<N2, N2> A = MatBuilder.fill(N2.instance, N2.instance, c1 * C1, c2 * C3, c2 * C1, c1 * C3);
        Matrix<N2, N2> B = MatBuilder.fill(N2.instance, N2.instance, c1 * C2, c2 * C4, c2 * C2, c1 * C4);
        Matrix<N2, N2> C = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);
        Matrix<N2, N2> D = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);

        double maxV =
            A.solve(B.times(VecBuilder.fill(12.0, 12.0))).times(-1.0).get(0, 0);
        System.out.println("Max V: " + maxV);

        return new LinearSystem<N2, N2, N2>(A, B, C, D);
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
        left.set(lSpeed);
        right.set(rSpeed);
    }

    public double[] setVelocity(double lVel, double rVel) {
        SmartDashboard.putNumber("Left target:", lVel);
        SmartDashboard.putNumber("Right target:", rVel);
        //velocity (m/s) / circumference of wheel * 4096 encoder units per rotation / 10 (u/100ms)
        double lWheelSpeed = (lVel * 409.6) / wheelCircumMeters;
        double rWheelSpeed = (rVel * 409.6) / wheelCircumMeters;
        double gearing = wheelCircumMeters / (409.6);

        //double controlL = left.calculate(left, lWheelSpeed);
        //double controlR = right.calculate(-rightFront.getSelectedSensorVelocity(), rWheelSpeed);
        //System.out.println("target: " + lWheelSpeed + " " + rWheelSpeed);
        //System.out.println("control: " + controlL + " " + controlR);

        //leftFront.set(TalonSRXControlMode.PercentOutput, -0.2);
        //leftBack.set(TalonSRXControlMode.PercentOutput, -0.2);
        //rightFront.set(TalonSRXControlMode.PercentOutput, controlR);
        //rightBack.set(TalonSRXControlMode.Follower, Constants.MotorCANValues.rightMotorFront);

        return null;
    }

    //DANGER
    public void setControl(double lSpeed, double rSpeed) {
        //DifferentialDriveWheelVoltages v = controller.calculate(new Pose2d(), leftFront.getSelectedSensorVelocity(0) * Math.PI * 0.0762 / 409.6, rightFront.getSelectedSensorVelocity(0) * Math.PI * 0.0762 / 409.6, new Pose2d(), lSpeed, rSpeed);
        //System.out.println("velocity: " + (leftFront.getSelectedSensorVelocity(0) * Math.PI * 0.0762 / 4096) + " " + (rightFront.getSelectedSensorVelocity(0) * Math.PI * 0.0762 / 4096));
        //System.out.println("control: " + v.left + " " + v.right);
        //set(v.left, v.right);
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
    }

    public void turnTo(Rotation2d rotation) {
        //angle.calculate(rotation.minus(estimator.getEstimatedPosition().getRotation()).getRadians());
    }

    int loop = 0;
    @Override
    public void periodic() {
        super.periodic();

        loop++;
        if (loop > 50) {
            System.out.println("imu yaw: " + imu.getAngle(IMUAxis.kYaw));
            loop = 0;
        }
        estimator.update(new Rotation2d(imu.getAngle(IMUAxis.kYaw) + Math.toRadians(180)), new DifferentialDriveWheelPositions(null, null));

        /*
        Optional<EstimatedRobotPose> pose = vision.getEstimatedGlobalPose(position);
        //System.out.println("periodic");
        if (pose.isPresent()) {
            System.out.println("Vision estimated pose:" + pose.get().estimatedPose.toString());
            position = pose.get().estimatedPose.toPose2d();
        }*/
    }

    public DifferentialDriveWheelSpeeds follow(Trajectory s, double t) {
        State p = s.sample(t);
        ChassisSpeeds c = ChassisSpeeds.fromFieldRelativeSpeeds(ramController.calculate(position, p), position.getRotation());
        c = new ChassisSpeeds(0.2, 0.2, 0);
        DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(c);
        System.out.println(
            "target velocities: "
             + (speeds.leftMetersPerSecond * 409.6) / wheelCircumMeters + " "
             + (speeds.rightMetersPerSecond * 409.6) / wheelCircumMeters + "\n" +
            "encoder targets: "
             + speeds.leftMetersPerSecond + " "
             + speeds.rightMetersPerSecond);
        double[] measured = setVelocity(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
        System.out.println("measured velocities: " + measured[0] + " " + measured[1] + "\n"
             + "measured encoder: " + measured[2] + " " + measured[3]);
        //add velocity references
        return speeds;
    }
}
