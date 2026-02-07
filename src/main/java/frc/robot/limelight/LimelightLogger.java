package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightLogger {

  private final String name;
  private final NetworkTable table;
  private double[] averages = new double[6];
  private boolean initialized = false;

  public LimelightLogger(String limelightName) {
    this.name = limelightName;
    this.table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  public void log() {
    double timestamp = Timer.getFPGATimestamp();

    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);

    // targetpose_cameraspace: [x, y, z, roll, pitch, yaw]
    double[] pose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    double x = pose[0];
    double y = pose[1];
    double z = pose[2];
    double roll = pose[3];
    double pitch = pose[4];
    double yaw = pose[5];

    double ambiguity = table.getEntry("ta").getDouble(1.0);

    if (!initialized) {
      averages = pose.clone();
    } else {
      for (int i = 0; i < 6; i++) {
        averages[i] = (averages[i] + pose[i]) / 2;
      }
    }

    SmartDashboard.putNumber(name + "/Average X", averages[0]);
    SmartDashboard.putNumber(name + "/Average Y", averages[1]);
    SmartDashboard.putNumber(name + "/Average Z", averages[2]);
    SmartDashboard.putNumber(name + "/Average Roll", averages[3]);
    SmartDashboard.putNumber(name + "/Average Pitch", averages[4]);
    SmartDashboard.putNumber(name + "/Average Yaw", averages[5]);

    // Push to SmartDashboard (easy CSV export)
    SmartDashboard.putNumber(name + "/Timestamp", timestamp);
    SmartDashboard.putNumber(name + "/TX", tx);
    SmartDashboard.putNumber(name + "/TY", ty);
    SmartDashboard.putNumber(name + "/X", x);
    SmartDashboard.putNumber(name + "/Y", y);
    SmartDashboard.putNumber(name + "/Z", z);
    SmartDashboard.putNumber(name + "/Roll", roll);
    SmartDashboard.putNumber(name + "/Pitch", pitch);
    SmartDashboard.putNumber(name + "/Yaw", yaw);
    SmartDashboard.putNumber(name + "/Ambiguity", ambiguity);
  }
}
