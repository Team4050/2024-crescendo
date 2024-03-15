package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class PathweaverTest extends Command {
    private Timer t;
    private int i = 0;
    private Trajectory paths[];
    private Drivetrain drive;

    public PathweaverTest(Drivetrain drive) {
        this.drive = drive;

        Path p1 = Filesystem.getDeployDirectory().toPath().resolve("output/1.wpilib.json");
        Path p2 = Filesystem.getDeployDirectory().toPath().resolve("output/2.wpilib.json");
        Path p3 = Filesystem.getDeployDirectory().toPath().resolve("output/3.wpilib.json");
        Path p4 = Filesystem.getDeployDirectory().toPath().resolve("output/4.wpilib.json");

        Trajectory t1;
        Trajectory t2;
        Trajectory t3;
        Trajectory t4;

        try {
            t1 = TrajectoryUtil.fromPathweaverJson(p1);
            t2 = TrajectoryUtil.fromPathweaverJson(p2);
            t3 = TrajectoryUtil.fromPathweaverJson(p3);
            t4 = TrajectoryUtil.fromPathweaverJson(p4);

            paths = new Trajectory[4];
            paths[0] = t1;
            paths[1] = t2;
            paths[2] = t3;
            paths[3] = t4;
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void initialize() {
        t = new Timer();
        t.start();
        Pose2d initial = paths[0].sample(0).poseMeters;
        System.out.println("Initial pose: " + initial);
        drive.setPose(initial);
    }

    @Override
    public void execute() {
        if (t.get() > paths[i].getTotalTimeSeconds() + 3.0) {
            i++;
            t.reset();
        }

        if (i < paths.length) {
            drive.follow(paths[i], t.get());
        }
    }

    @Override
    public boolean isFinished() {
        return i >= paths.length;
    }

    @Override
    public void end(boolean interrupted) {
        drive.set(0, 0);
    }
}
