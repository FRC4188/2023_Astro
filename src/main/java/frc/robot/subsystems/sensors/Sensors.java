package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Limelight limelightLeft = new Limelight(Constants.sensors.LEFT_NAME, Constants.sensors.LEFT_POSITION);
  private Limelight limelightRight = new Limelight(Constants.sensors.RIGHT_NAME, Constants.sensors.RIGHT_POSITION);

  /** Creates a new Sensors. */
  private Sensors() {}

  private void init() {}

  @Override
  public void periodic() {  
    SmartDashboard.putString("Estimated Pose", getPose2d().toString());

    // testing 
    // SmartDashboard.putString("Left Estimated Pose", limelightLeft.getPose3d().toString());
    // SmartDashboard.putString("Right Estimated Pose", limelightRight.getPose3d().toString());
  }

  public Pose3d getPose3d() {
    Pose3d poseLeft = limelightLeft.getPose3d();
    Pose3d poseRight = limelightRight.getPose3d();
    Pose3d poseEmpty = new Pose3d(); 

    if(!poseLeft.equals(poseEmpty) && !poseRight.equals(poseEmpty)) {

      // averages the poses of both limelights
      return new Pose3d(
        poseLeft.getTranslation().plus(poseRight.getTranslation()).div(2),
        poseLeft.getRotation().plus(poseRight.getRotation()).div(2)
        
      );
    } else if(poseLeft.equals(poseEmpty)) {
      return poseRight;
    } else if(poseRight.equals(poseEmpty)) {
      return poseLeft;
    } else return poseEmpty;
  }
 
  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  // public double getLatency() {
  //   return something;
  // }
}
