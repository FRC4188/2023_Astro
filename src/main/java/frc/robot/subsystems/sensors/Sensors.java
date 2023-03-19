package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Pigeon pigeon = new Pigeon(Constants.ids.PIGEON);
  private Limelights limelights = new Limelights("limelight-front", "limelight-back");

  /** Creates a new Sensors. */
  private Sensors() {}

  private void init() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon Angle", getRotation2d().getDegrees());
    SmartDashboard.putString("Estimated Pose", getPose3d().toString());
  }

  public Pose3d getPose3d() {
    return limelights.getPose3d();
  }

  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  public double getLatency() {
    return limelights.getLatency();
  }

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public void resetPigeon() {
    pigeon.reset();
  }
}
