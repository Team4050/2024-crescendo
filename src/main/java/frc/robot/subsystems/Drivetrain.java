package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardVision;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Drivetrain extends SubsystemBase {
  /* Motors */
  private CANSparkMax leftF;
  private CANSparkMax leftR;
  private CANSparkMax rightF;
  private CANSparkMax rightR;

  /* Controls */
  private LTVDifferentialDriveController controller;
  private RamseteController ramController;

  private DifferentialDrive drivetrain;
  private Pose2d position;
  private DifferentialDrivePoseEstimator estimator;

  private SparkPIDController leftControl;
  private SparkPIDController rightControl;

  /* Sensors */
  private HazardVision vision;
  private boolean useVision = false;
  // 1.15 degree drift over 2 and a half minutes
  private ADIS16470_IMU imu;
  private boolean useIMU = true;

  /* Constants */
  private double wheelRadMeters = 0.1524;
  private double wheelCircumMeters = wheelRadMeters * Math.PI * 2;
  private double encoderToMeters = wheelCircumMeters / 42; // TODO:
  private double gearRatio = 8.46;
  private DifferentialDriveKinematics kinematics;

  private double kP = 0.000;
  private double kI = 0;
  private double kD = 0;
  private double kF = 0;

  public Drivetrain(Pose2d initial) {
    leftF = new CANSparkMax(Constants.MotorCANValues.leftMotorFront, MotorType.kBrushless);
    leftR = new CANSparkMax(Constants.MotorCANValues.leftMotorRear, MotorType.kBrushless);
    rightF = new CANSparkMax(Constants.MotorCANValues.rightMotorFront, MotorType.kBrushless);
    rightR = new CANSparkMax(Constants.MotorCANValues.rightMotorRear, MotorType.kBrushless);

    leftR.follow(leftF);
    rightR.follow(rightF);

    rightF.setInverted(true);
    leftF.setInverted(false);

    leftF.setIdleMode(IdleMode.kBrake);
    leftR.setIdleMode(IdleMode.kBrake);
    rightF.setIdleMode(IdleMode.kBrake);
    rightR.setIdleMode(IdleMode.kBrake);

    leftControl = leftF.getPIDController();
    rightControl = rightF.getPIDController();

    ramController = new RamseteController(1, 0.5);

    leftControl.setP(kP);
    leftControl.setI(kI);
    leftControl.setD(kD);
    leftControl.setFF(kF);

    kinematics = new DifferentialDriveKinematics(0.508);
    estimator =
        new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(0), 0, 0, initial);
    position = initial;
    if (useVision) {
      vision = new HazardVision();
    } else {
      vision = null;
    }
    if (useIMU) {
    imu = new ADIS16470_IMU();
    imu.configCalTime(ADIS16470_IMU.CalibrationTime._128ms);
    imu.calibrate();
    }
    else {
      imu = null;
    }
    Vector<N5> qelems = VecBuilder.fill(0.2, 0.2, 0.2, 0.2, 0.2);
    Vector<N2> relems = VecBuilder.fill(0.01, 0.01);

    Matrix<N2, N2> A = MatBuilder.fill(N2.instance, N2.instance, -0.2, 0.1, 0.1, -0.2);
    Matrix<N2, N2> B = MatBuilder.fill(N2.instance, N2.instance, 0.2, -0.1, -0.1, 0.2);

    double maxV = A.solve(B.times(VecBuilder.fill(12.0, 12.0))).times(-1.0).get(0, 0);
    System.out.println(maxV);

    // Gear ratio for 2022 was 10.75:1
    LinearSystem<N2, N2, N2> plant =
        buildPlant(0.0762, 0.27305, 0.118, 0.118, 5.0, 40.0, 1, 1, 1, 1);

    // track width ~20 inch?
    controller = new LTVDifferentialDriveController(plant, 0.508, qelems, relems, 0.02);

    ramController = new RamseteController(0.5, 0.2);
  }

  private void configMotor(TalonSRX m) {
    m.config_kP(0, 0);
    m.config_kI(0, 0);
    m.config_kD(0, 0);
    m.config_kF(0, 0);
  }

  private LinearSystem<N2, N2, N2> buildPlant(
      double r,
      double rb,
      double Gl,
      double Gr,
      double J,
      double m,
      double Kt,
      double Kv,
      double Rl,
      double Rr) {
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
     * J = moment of inertia  m = robot mass TODO:figure out mass of current robot, 80lbs limit, 36.28736kg
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

    Matrix<N2, N2> A =
        MatBuilder.fill(N2.instance, N2.instance, c1 * C1, c2 * C3, c2 * C1, c1 * C3);
    Matrix<N2, N2> B =
        MatBuilder.fill(N2.instance, N2.instance, c1 * C2, c2 * C4, c2 * C2, c1 * C4);
    Matrix<N2, N2> C = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);
    Matrix<N2, N2> D = MatBuilder.fill(N2.instance, N2.instance, 1, 0, 0, 1);

    double maxV = A.solve(B.times(VecBuilder.fill(12.0, 12.0))).times(-1.0).get(0, 0);
    System.out.println("Max V: " + maxV);

    return new LinearSystem<N2, N2, N2>(A, B, C, D);
  }

  public Drivetrain() {
    this(new Pose2d());
  }

  /** Get the robot's speed and direction */
  public Pose2d get() {
    if (useVision) {
        Optional<EstimatedRobotPose> o = vision.getEstimatedGlobalPose(position);
        if (!o.isEmpty()) {
        position = o.get().estimatedPose.toPose2d();
      }
    }
    return position;
  }

  /**
   * Speed in percent output
   *
   * @param lSpeed
   * @param rSpeed
   */
  public void set(double lSpeed, double rSpeed) {
    leftF.set(lSpeed);
    // leftR.set(lSpeed);
    rightF.set(rSpeed);
    // rightR.set(rSpeed);
  }

  /**
   * Set voltage output
   *
   * @param lV
   * @param rV
   */
  public void setV(double lV, double rV) {
    leftF.setVoltage(lV);
    leftR.setVoltage(lV);
    rightF.setVoltage(rV);
    rightR.setVoltage(rV);
  }

  public double[] setVelocity(double lVel, double rVel) {
    //SmartDashboard.putNumber("Left target:", lVel);
    //SmartDashboard.putNumber("Right target:", rVel);
    // velocity (m/s) / circumference of wheel * 4096 encoder units per rotation / 10 (u/100ms)
    double lEncoderSpeed = (lVel * gearRatio * 42) / wheelCircumMeters;
    double rEncoderSpeed = (rVel * gearRatio * 42) / wheelCircumMeters;
    double gearing = wheelCircumMeters;

    System.out.println(lEncoderSpeed + " " + rEncoderSpeed);

    leftControl.setReference(lEncoderSpeed, ControlType.kVelocity);
    rightControl.setReference(lEncoderSpeed, ControlType.kVelocity);

    // double controlL = left.calculate(left, lWheelSpeed);
    // double controlR = right.calculate(-rightFront.getSelectedSensorVelocity(), rWheelSpeed);
    // System.out.println("target: " + lWheelSpeed + " " + rWheelSpeed);
    // System.out.println("control: " + controlL + " " + controlR);

    return null;
  }

  int loop = 0;

  @Override
  public void periodic() {
    super.periodic();

    //loop++;
    /*if (loop > 200) {
      System.out.println("imu yaw: " + imu.getAngle(IMUAxis.kYaw));
      loop = 0;
    }*/

    if (useIMU) {
    estimator.update(
        new Rotation2d(imu.getAngle(IMUAxis.kYaw) + Math.toRadians(180)),
        new DifferentialDriveWheelPositions(
            leftF.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * encoderToMeters,
            rightF.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * encoderToMeters));
    }

    /*
    Optional<EstimatedRobotPose> pose = vision.getEstimatedGlobalPose(position);
    //System.out.println("periodic");
    if (pose.isPresent()) {
        System.out.println("Vision estimated pose:" + pose.get().estimatedPose.toString());
        position = pose.get().estimatedPose.toPose2d();
    }*/
  }

  public boolean follow(Trajectory s, double t) {
    State p = s.sample(t);
    /*DifferentialDriveWheelVoltages dv =
        controller.calculate(
            position,
            leftF.getAbsoluteEncoder(Type.kDutyCycle).getVelocity() * encoderToMeters,
            rightF.getAbsoluteEncoder(Type.kDutyCycle).getVelocity() * encoderToMeters,
            p);*/
    ChassisSpeeds speeds = ramController.calculate(position, p);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    setVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    return ramController.atReference();
  }
}
