package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Pigeon pigeon = new Pigeon(Constants.ids.PIGEON);
  
  private Limelight limelightLeft =
      new Limelight(
          Constants.sensors.LEFT_NAME,
          Constants.sensors.LEFT_POSITION,
          Constants.sensors.LEFT_ROTATION);
  private Limelight limelightRight =
      new Limelight(
          Constants.sensors.RIGHT_NAME,
          Constants.sensors.RIGHT_POSITION,
          Constants.sensors.RIGHT_ROTATION);



  /** Creates a new Sensors. */
  private Sensors() {}

  private void init() {}

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Pigeon Angle", getRotation2d().getDegrees());
    // SmartDashboard.putString("Limelight Pose3d", getPose3d().toString());
    // SmartDashboard.putString("Limelight Pose2d", getPose2d().toString());
    // SmartDashboard.putString("Left Pose", limelightLeft.getPose3d().toString());
    // SmartDashboard.putString("Right Pose", limelightRight.getPose3d().toString());

  }

  public Pose3d getPose3d() {
    if (limelightRight.getTV() && limelightLeft.getTV()) {
      Pose3d poseLeft = limelightLeft.getPose3d();
      Pose3d poseRight = limelightRight.getPose3d();

      // averages the poses of both limelights
      return new Pose3d(
          poseLeft.getTranslation().plus(poseRight.getTranslation()).div(2),
          poseLeft.getRotation().plus(poseRight.getRotation()).div(2));
    } else if (limelightRight.getTV()) {
      return limelightRight.getPose3d();
    } else if (limelightLeft.getTV()) {
      return limelightLeft.getPose3d();
    } else return new Pose3d();
  }

  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  public double getLatency() {
    if (limelightRight.getTV() && limelightLeft.getTV()) {
      // averages the latencies
      return (limelightRight.getLatency() + limelightLeft.getLatency()) / 2;
    } else if (limelightRight.getTV()) {
      return limelightRight.getLatency();
    } else if (limelightLeft.getTV()) {
      return limelightLeft.getLatency();
    } else return 0.0;
  }

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public void resetPigeon() {
    setPigeonAngle(new Rotation2d());
  }

  public void setPigeonAngle(Rotation2d angle) {
    pigeon.setYaw(angle.getDegrees());
    Drivetrain.getInstance().setRotSetpoint(angle.getDegrees());
  }
}
