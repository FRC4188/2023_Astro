// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import csplib.utils.LimelightHelpers;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Add your docs here. */
public class Limelights {
  private String frontLLName;
  private String backLLName;

  private MedianFilter filter = new MedianFilter(2);

  public Limelights(String frontLLName, String backLLName) {
    this.frontLLName = frontLLName;
    this.backLLName = backLLName;

    init();
  }

  private void init() {
    LimelightHelpers.setCameraPose_RobotSpace(
        frontLLName,
        Constants.sensors.FRONT_POSITION.getX(),
        Constants.sensors.FRONT_POSITION.getY(),
        Constants.sensors.FRONT_POSITION.getZ(),
        0,
        0,
        0);

    LimelightHelpers.setCameraPose_RobotSpace(
        backLLName,
        Constants.sensors.BACK_POSITION.getX(),
        Constants.sensors.BACK_POSITION.getY(),
        Constants.sensors.BACK_POSITION.getZ(),
        0,
        0,
        180);
  }

  private Pose3d filterPose(Pose3d pose) {
    double filteredX = filter.calculate(pose.getX());
    double filteredY = filter.calculate(pose.getY());
    double filteredZ = filter.calculate(pose.getZ());

    return new Pose3d(filteredX, filteredY, filteredZ, pose.getRotation());
  }

  public Pose3d getPose3d() {
    if (LimelightHelpers.getTV(frontLLName)) {
      return (DriverStation.getAlliance() == Alliance.Blue)
          ? filterPose(LimelightHelpers.getBotPose3d_wpiBlue(frontLLName))
          : filterPose(LimelightHelpers.getBotPose3d_wpiRed(frontLLName).relativeTo(getPose3d()));
    } else if (LimelightHelpers.getTV(backLLName)) {
      return filterPose(LimelightHelpers.getBotPose3d_wpiBlue(backLLName));
    } else return new Pose3d();
  }

  public double getLatency() {
    double time = Timer.getFPGATimestamp();
    if (LimelightHelpers.getTV(frontLLName)) {
      return time
          - (LimelightHelpers.getLatency_Capture(frontLLName) / 1000)
          - (LimelightHelpers.getLatency_Pipeline(frontLLName) / 1000);
    } else if (LimelightHelpers.getTV(backLLName)) {
      return time
          - (LimelightHelpers.getLatency_Capture(backLLName) / 1000)
          - (LimelightHelpers.getLatency_Pipeline(backLLName) / 1000);
    } else return 0.0;
  }
}
