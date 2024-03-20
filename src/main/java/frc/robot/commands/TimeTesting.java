package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class TimeTesting extends Command {
    private Drivetrain drivetrain;
    private Intake intake;
    private Timer timer;
    private double time;
    private double direction = 1;
    private boolean turning;
    private boolean note;
    private static final double turningConstant = 0.00355;
    
    // 45 is 30 degrees
    // 78 is 45
    // 84 is 60
    // 120 is 90

    public TimeTesting(Drivetrain drivetrain, Intake intake, double value, boolean turning, boolean note) {
        this.drivetrain = drivetrain;
        this.turning = turning;
        if (value < 0) {
            direction = -1;
        }
        time = getTime(Math.abs(value));
        if (turning) {
            time *= turningConstant;
        }
        this.intake = intake;
        this.note = note;
    }

    @Override
    public void initialize() {
        System.out.println("Begin sequence");
        timer = new Timer();
        timer.start();
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        if (timer.get() > time) {
            drivetrain.set(0, 0);
        }
        else {
        if (turning) {
            System.out.println(0.8 * direction);
            drivetrain.set(0.5 * direction, -0.5 * direction);
            return;
        } 
        drivetrain.set(0.5 * direction, 0.5 * direction);
        }

        if (note) {
            intake.setIntake(false, true);
        }

        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(false, false);
        drivetrain.set(0, 0);
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        HashSet<Subsystem> reqs = new HashSet<Subsystem>();
        reqs.add(drivetrain);
        reqs.add(intake);
        return reqs;
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.get() > time + 0.5;
    }

    // Model drivetrain movement linearly
    private double m = 1.0;
    private double b = 0.0;
    private double getTime(double distance) {
        return (m * distance) + b; 
    }
}
