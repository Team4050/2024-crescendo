package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class TimeTesting extends Command {
    private Drivetrain drivetrain;
    private Timer timer;
    private double time;
    private double direction = 1;
    private boolean turning;
    private static final double turningConstant = 0.00355;

    public TimeTesting(Drivetrain drivetrain, double value, boolean turning) {
        this.drivetrain = drivetrain;
        this.turning = turning;
        if (value < 0) {
            direction = -1;
        }
        time = Math.abs(value);
        if (turning) {
            time *= turningConstant;
        }
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

        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        HashSet<Subsystem> reqs = new HashSet<Subsystem>();
        reqs.add(drivetrain);
        return reqs;
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.get() > time + 3;
    }
}
