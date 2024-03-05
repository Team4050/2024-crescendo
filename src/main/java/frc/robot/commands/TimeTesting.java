package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TimeTesting extends Command {
    private Drivetrain drivetrain;
    private Timer timer;
    private double time;
    private double direction = 1;
    private boolean turning;
    private static final double turningConstant = 0.01;

    public TimeTesting(Drivetrain drivetrain, double value, boolean turning) {
        this.drivetrain = drivetrain;
        this.turning = turning;
        if (value < 0) {
            direction = -1;
        }
        time = Math.abs(value);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        if (turning) {
            drivetrain.set(0.8 * direction, -0.8 * direction);
            super.execute();
            return;
        } 
        drivetrain.set(0.8 * direction, 0.8 * direction);
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.get() > time;
    }
}
