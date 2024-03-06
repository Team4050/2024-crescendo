package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.Drivetrain;

public class Profiling extends Command {
    private Drivetrain drivetrain;
    private HazardXbox controller;
    private Timer timer;

    private double[] xl = new double[8192];
    private double[] yl = new double[8192];
    private double[] xr = new double[8192];
    private double[] yr = new double[8192];
    private int samplesl = 0;
    private int samplesr = 0;

    private double voltage = 0;

    public Profiling(Drivetrain drivetrain, HazardXbox controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("Start drivetrain profiling..");
        timer.start();
        System.out.println("Friction test: put drivetrain on blocks");
        // TODO Auto-generated method stub
        super.initialize();
    }

    // Quasi-static determination 
    // https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915
    // Section 4.2
    //
    int loop = 0;
    @Override
    public void execute() {
        voltage += 0.003;
        loop++;
        if (loop > 50) {
            System.out.println("Current voltage: " + voltage);
            loop = 0;
        }
        double clamp = voltage;
        if (clamp > 12) {
            clamp = 12;
        }

        clamp = 12;

        drivetrain.setV(clamp, clamp);
        //drivetrain.set(clamp / 12, clamp / 12);

        double[] encoderSpeeds = drivetrain.getEncoderV();

        xl[samplesl] = voltage;
        yl[samplesl] = encoderSpeeds[0] / 42;
        samplesl++;

        xr[samplesr] = voltage;
        yr[samplesr] = encoderSpeeds[1] / 42;
        samplesr++;

        //System.out.println();
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // A = sumy * sumxsq - sumx * sumxy / samplesl * sumxsq - sumx^2
        //
        // B = samples * sumxy - sumx * sumy / samples * sumxsq - sumx^2
        //
        // y = A + Bx
        double Al, Bl, Ar, Br;

        double sumxl = 0;
        double sumyl = 0;
        double sumxsql = 0;
        double sumxyl = 0;

        double sumxr = 0;
        double sumyr = 0;
        double sumxsqr = 0;
        double sumxyr = 0;

        for (int i = 0; i < samplesl; i++) {
            sumxl += xl[i];
            sumyl += yl[i];
            sumxsql += Math.pow(xl[i], 2);
        }

        for (int i = 0; i < samplesr; i++) {
            sumxr += xr[i];
            sumyr += yr[i];
            sumxsqr += Math.pow(xr[i], 2);
        }

        Al = (sumyl * sumxsql) - (sumxl * sumxyl) / ((samplesl * sumxsql) - Math.pow(sumxl, 2));
        Bl = (samplesl * sumxyl) - (sumxl * sumyl) / ((samplesl * sumxsql) - Math.pow(sumxl, 2));

        Ar = (sumyr * sumxsqr) - (sumxr * sumxyr) / ((samplesr * sumxsqr) - Math.pow(sumxr, 2));
        Br = (samplesr * sumxyr) - (sumxr * sumyr) / ((samplesr * sumxsqr) - Math.pow(sumxr, 2));

        System.out.println("Regression results:");
        System.out.println("Left A, B: " + Al + " " + Bl);
        System.out.println("Right A, B: " + Ar + " " + Br);
        System.out.println("L, R X intercept: " + (Al / Bl) + " " + (Ar / Br));

        drivetrain.set(0, 0);

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
        return voltage > 12.5;
    }
}