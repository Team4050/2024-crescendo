package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Gun;
import frc.robot.subsystems.Intake;

public class Shoot extends Command {
    private Gun shooter;
    private Intake intake;
    private Timer timer;

    public Shoot(Gun shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        timer = new Timer();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> reqs = new HashSet<Subsystem>();
        reqs.add(shooter);
        reqs.add(intake);
        return reqs;
    }

    @Override
    public void initialize() {
        timer.start();
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setConveyor(1);
        shooter.fire(1);
        super.execute();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.get() > 6.0;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        intake.setConveyor(0);
        shooter.fire(0);
        super.end(interrupted);
    }
}
