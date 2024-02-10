package frc.robot.hazard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class HazardCamera {
    private NetworkTable limelight1;

    private NetworkTableEntry noteX;
    private NetworkTableEntry noteY;

    public HazardCamera() {
        limelight1 = NetworkTableInstance.getDefault().getTable(Constants.Sensors.IntakeCameraName);
        
        noteX = limelight1.getEntry("tx");
        noteY = limelight1.getEntry("ty");
    }

    public Transform2d getPosition() {
        SmartDashboard.putNumber("LimelightX", noteX.getDouble(0));
        SmartDashboard.putNumber("LimelightY", noteY.getDouble(0));
        System.out.println("Limelight intake X: " + noteX.getDouble(0));
        return new Transform2d(noteX.getDouble(0), noteY.getDouble(0), new Rotation2d());
    }
}