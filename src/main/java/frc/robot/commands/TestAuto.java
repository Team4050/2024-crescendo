package frc.robot.commands;

import java.util.List;
import java.util.ArrayList;
import java.util.Set;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class TestAuto extends Command {
    private Drivetrain drive;
    private Timer timer;
    private Trajectory trajectory;

    public TestAuto(Drivetrain d) {
        drive = d;
    }

    @Override
    public void initialize() {
        super.initialize();
        timer = new Timer();
        timer.start();
        TrajectoryConfig tconfig = new TrajectoryConfig(Units.MetersPerSecond.of(1), Units.MetersPerSecondPerSecond.of(10));
        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
            tconfig);
    }

    @Override
    public void execute() {
        DifferentialDriveWheelVoltages d = drive.follow(trajectory, timer.get());
        drive.setControl(1, 1);
        double end = timer.get() + 1;
        while (timer.get() < end) {};
        drive.setControl(0, 0);
        System.out.println("Wheel voltages: " + d.left + " " + d.right);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return super.getRequirements();
    }
}
