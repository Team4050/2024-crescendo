package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
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
  private IdleMode idleMode = IdleMode.kBrake;

  private LTVDifferentialDriveController controller;
  private LinearSystem<N2, N2, N2> drivetrainModel;
  private RamseteController ramController;

  private DifferentialDrive drivetrain;
  private Pose2d position;
  private DifferentialDrivePoseEstimator estimator;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkPIDController leftControl;
  private SparkPIDController rightControl;

  /* Sensors */
  private HazardVision vision;
  private boolean useVision = false;
  // 1.15 degree drift over 2 and a half minutes
  private ADIS16470_IMU imu;
  private boolean useIMU = true;

  private CANcoder leftExternalEncoder;
  private CANcoder rightExternalEncoder;

  private double LEMA = 0;
  private double REMA = 0;

  private double alpha = 0.06;

  /* Constants */
  public DifferentialDriveKinematics kinematics;
  //Robot weight is 95.7lbs

  //
  private double kP = 0.002;
  private double kI = 0;
  private double kD = 0.0000001;
  private double kF = 0.00004;

  public Drivetrain(Pose2d initial) {
    leftF = new CANSparkMax(Constants.MotorCANValues.leftMotorFront, MotorType.kBrushless);
    leftR = new CANSparkMax(Constants.MotorCANValues.leftMotorRear, MotorType.kBrushless);
    rightF = new CANSparkMax(Constants.MotorCANValues.rightMotorFront, MotorType.kBrushless);
    rightR = new CANSparkMax(Constants.MotorCANValues.rightMotorRear, MotorType.kBrushless);

    leftR.follow(leftF);
    rightR.follow(rightF);

    rightF.setInverted(true);
    rightR.setInverted(true);
    leftF.setInverted(false);
    leftR.setInverted(false);

    leftF.setIdleMode(idleMode);
    leftR.setIdleMode(idleMode);
    rightF.setIdleMode(idleMode);
    rightR.setIdleMode(idleMode);

    leftEncoder = leftF.getEncoder();
    rightEncoder = rightF.getEncoder();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftControl = leftF.getPIDController();
    rightControl = rightF.getPIDController();

    // TODO: Attach external encoders
    // leftExternalEncoder = new CANcoder(Constants.MotorCANValues.cancoderLeft);
    // rightExternalEncoder = new CANcoder(Constants.MotorCANValues.cancoderRight);

    leftControl.setP(kP);
    leftControl.setI(kI);
    leftControl.setD(kD);
    leftControl.setFF(kF);

    rightControl.setP(kP);
    rightControl.setI(kI);
    rightControl.setD(kD);
    rightControl.setFF(kF);

    kinematics = new DifferentialDriveKinematics(RobotConstants.robotTrackWidth);
    position = initial;
    if (useVision) {
      vision = new HazardVision();
    } else {
      vision = null;
    }
    if (useIMU) {
      imu = new ADIS16470_IMU();
      imu.configCalTime(ADIS16470_IMU.CalibrationTime._4s);
      imu.calibrate();
    }
    else {
      imu = null;
    }
    // X, Y, Rotation in radians, XVel, YVel
    //Vector<N5> qelems = VecBuilder.fill(0.08, 0.08, Math.PI / 70, 0.15, 0.15);
    //Vector<N2> relems = VecBuilder.fill(0.5, 0.5);

    /*
     * Monday night testing results: not good. Never reaches setpoints, jitters and overcorrects on straight lines.
     * This means two things: 1) The system is never reaching angle and velocity setpoints when the running appears smooth 2) The system is not properly tuned to converge to a setpoint when it reaches it
     * Increasing the control effort won't fix this, neither will adjusting tolerances.
     * The problem, therefore, is in the system's model
     * I never got a fully confirmed calculation on the motor resistances.
     * 12V/60A (Smart limit) = 0.2
     * 
     * Found 1 issue, velocities were being fed as RPM, instead of ticks/s (fixed)
     * Related to that, initial encoder positions were not scaled by encoder ticks, causing a jerk when starting auto (fixed)
     * Limelight usage causes pose estimator to be thrown off, and reports impossible positions due to april tags not being mounted securely
     * Does not turn in place
     */
    Vector<N5> qelems = VecBuilder.fill(0.08, 0.08, Math.PI / 70, 0.15, 0.15);
    Vector<N2> relems = VecBuilder.fill(0.3, 0.3);


    // Gear ratio for 2022 (Omega) was 10.75:1. For 2024, it is 8.45:1
    // Moment of inertia based on https://en.wikipedia.org/wiki/List_of_moments_of_inertia#Moments_of_inertia (wikipedia as source lol),
    // Solid rectangular cuboid model:
    // 1/12 * m * (h^2 + w^2)
    // TODO: find L & R resistance R = V/I (stall) = 12V/100A = 0.12?
    System.out.println(0.08333 * RobotConstants.robotMassKG * (Math.pow(RobotConstants.robotWidth, 2) + Math.pow(RobotConstants.robotDepth, 2)));
    drivetrainModel =
        buildPlant(Constants.RobotConstants.wheelRadMeters, 
            0.27305,
            RobotConstants.gearRatio, 
            RobotConstants.gearRatio, 
            0.08333 * RobotConstants.robotMassKG * (Math.pow(RobotConstants.robotWidth, 2) + Math.pow(RobotConstants.robotDepth, 2)), 
            RobotConstants.robotMassKG, 
            50.265,//96.028, 
            0.2, 
            0.2);

    controller = new LTVDifferentialDriveController(drivetrainModel, RobotConstants.robotTrackWidth, qelems, relems, 0.02);

    estimator =
        new DifferentialDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            leftEncoder.getPosition() * 42 * RobotConstants.encoderToMeters,
            rightEncoder.getPosition() * 42 * RobotConstants.encoderToMeters,
            initial);

    // Ramsete controller tuning for meters is 2.0 & 0.7
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    ramController = new RamseteController(1.6, 0.7);
  }

  /**
   * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#Hfootnote.27
   * State vector (velocities)
   * x = [ vl ]
   *     [ vr ]
   *
   * Input vec (Voltages)
   * u = [ Vl ]
   *     [ Vr ]
   *
   * @param r = wheel radius  
   * @param rb = wheelbase radius 
   * @param Gl = gear ratio between left wheels and motor
   * @param Gr = gear ration between right wheels and motor
   * @param J = moment of inertia  
   * @param m = robot mass
   * @param Kt = motor torque constant
   * @param Kv = free spinning rpm at 1 V
   * @param Rl = left motor resistance
   * @param Rr = right motor resistance
   * 
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
   * 
   * Kv for NEO is 917 under no load, 917 in RPM * 2 * PI * / 60 = 96.028
   * However, in the SparkMax, the Kv is set to 480.
   * https://www.brushless.com/how-to-calculate-the-kv-value-of-brushless-motor
   * https://www.revrobotics.com/content/docs/REV-21-1651-DS.pdf
   * 
   * determining Kt is stall torque over stall current
   * Kt = 60 / 2pi * Kv  Kt = T / I = 0.0108
   * https://motors.vex.com/other-motors/neo550
   * https://electronics.stackexchange.com/questions/531406/how-to-calculate-the-torque-constant-for-a-bldc-motor
   * @return A LinearSystem that represents the drivetrain
   */
  private LinearSystem<N2, N2, N2> buildPlant(
      double r,
      double rb,
      double Gl,
      double Gr,
      double J,
      double m,
      double Kv,
      double Rl,
      double Rr) {
    /*
     */
    double Kt = 0.97 / 100;//1.08 * 2 / 111;//60 / (2 * Math.PI * Kv);
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
    return position;
  }

  public double[] getEncoderV() {
    return new double[] {LEMA, REMA};
  }

  /**
   * Speed in percent output
   *
   * @param lSpeed
   * @param rSpeed
   */
  public void set(double lSpeed, double rSpeed) {
    leftF.set(lSpeed);
    rightF.set(rSpeed);
  }

  // setVoltage and setReference should be the same thing, but the follower API may be causing issues
  // https://www.chiefdelphi.com/t/sparkmax-api-set-current-set-voltage-calls/385160/7
  public void setV(double lVolts, double rVolts) {
    leftControl.setReference(lVolts, ControlType.kVoltage);
    rightControl.setReference(rVolts, ControlType.kVoltage);
  }

  public void setVcontrol(double lVolts, double rVolts) {
    leftControl.setReference(lVolts, ControlType.kVoltage);
    rightControl.setReference(rVolts, ControlType.kVoltage);
  }

  /**
   * Align the robot shooter to face a point at a set distance from it
   *
   * @param point
   * @param distance
   */
  public void align(Translation2d point, double distance) {}

  public Optional<EstimatedRobotPose> getRawCameraDetection() {
    return vision.getEstimatedGlobalPose(position);
  }

  public Rotation2d getTransformedIMU() {
    if (useIMU) {
      return new Rotation2d(Math.toRadians(imu.getAngle(IMUAxis.kRoll)));
    }
    return new Rotation2d();
  }

  /**
   * Set IMU angle in degrees. This method does nothing if the IMU is disabled
   * @param yaw
   */
  public void setIMU(double yaw) {
    System.out.println("Reset IMU to: " + yaw);
    if (useIMU) {
      imu.setGyroAngle(Constants.Sensors.robotYawAxis, yaw);
    }
  }

  // TODO: Test if issues with pose resetting have been fixed
  public void setPose(Pose2d manual) {
    setIMU(manual.getRotation().getDegrees());
    estimator.resetPosition(getTransformedIMU(), leftEncoder.getPosition() * 42 * RobotConstants.encoderToMeters, rightEncoder.getPosition() * 42 * RobotConstants.encoderToMeters, manual);
    update();
    System.out.println("Reset position to: " + position);
  }

  int loop = 0;

  @Override
  public void periodic() {
    super.periodic();

    // Bruh why am I blind
    LEMA = LEMA + alpha * ((leftEncoder.getVelocity() * 42 / 60) - LEMA);
    REMA = REMA + alpha * ((rightEncoder.getVelocity() * 42 / 60) - REMA);

    loop++;
    if (loop > 150) {
      /*System.out.println("LeftF duty cycle, bus voltage: " + leftF.getAppliedOutput() + " " + leftF.getBusVoltage());
      System.out.println("LeftR duty cycle, bus voltage: " + leftR.getAppliedOutput() + " " + leftR.getBusVoltage());
      System.out.println("RightF duty cycle, bus voltage: " + rightF.getAppliedOutput() + " " + rightF.getBusVoltage());
      System.out.println("RightR duty cycle, bus voltage: " + rightR.getAppliedOutput() + " " + rightR.getBusVoltage());
      */
      //System.out.println("L, R: " + ((leftEncoder.getVelocity() * Constants.RobotConstants.encoderToMeters)) + " " + ((rightEncoder.getVelocity() * Constants.RobotConstants.encoderToMeters)));
      //System.out.println("Pos: " + position.getX());
      //System.out.println(getEncoderV()[0] + " " + getEncoderV()[1]);
      System.out.println(position);
      System.out.println("imu yaw: " + getTransformedIMU());
      loop = 0;
    }

    update();
  }

  public void update() {
    position =
        estimator.update(
            getTransformedIMU(),
            new DifferentialDriveWheelPositions(
                leftEncoder.getPosition() * 42 * RobotConstants.encoderToMeters,
                rightEncoder.getPosition() * 42 * RobotConstants.encoderToMeters));

    if (useVision) {
      Optional<EstimatedRobotPose> pose = vision.getEstimatedGlobalPose(position);
      if (pose.isPresent()) {
          System.out.println("Vision estimated pose:" + pose.get().estimatedPose.toString());
          estimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
          setIMU(position.getRotation().getDegrees());
      }
    }
  }

  public boolean follow(Trajectory s, double t) {
    State p = s.sample(t);

    DifferentialDriveWheelVoltages v = controller.calculate(position, LEMA * Constants.RobotConstants.encoderToMeters, REMA * Constants.RobotConstants.encoderToMeters, p);
    //System.out.println("err: " + (p.velocityMetersPerSecond - (((leftEncoder.getVelocity() * Constants.RobotConstants.encoderToMeters) + (rightEncoder.getVelocity() * Constants.RobotConstants.encoderToMeters)) / 2)));
    setV(v.left, v.right);
    return controller.atReference();
  }

  // TODO: method of direct use of controller for speaker and amp alignment?
  public void control(Pose2d targetPose, double lVel, double rVel) {
    DifferentialDriveWheelVoltages v = controller.calculate(position, LEMA * Constants.RobotConstants.encoderToMeters, REMA * Constants.RobotConstants.encoderToMeters, targetPose, lVel, rVel);
    setV(v.left, v.right);
  }
}
