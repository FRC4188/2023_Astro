// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import csplib.utils.LimelightHelpers;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Add your docs here. */
public class Limelight {
  private String name;
  private Translation3d position = new Translation3d();

  // private MedianFilter filter = new MedianFilter(2);

  public Limelight(String name, Translation3d position) {
    this.name = name;
    this.position = position;
    init();
  }

  private void init() {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        position.getX(),
        position.getY(),
        position.getZ(),
        0,
        0,
        0);
  }

  // private Pose3d filterPose(Pose3d pose) {
  //   double filteredX = filter.calculate(pose.getX());
  //   double filteredY = filter.calculate(pose.getY());
  //   double filteredZ = filter.calculate(pose.getZ());

  //   return new Pose3d(filteredX, filteredY, filteredZ, pose.getRotation());
  // }

  // public Pose3d getPose3d() {
  //   if (LimelightHelpers.getTV(name)) {
  //     return (DriverStation.getAlliance() == Alliance.Blue)
  //         ? filterPose(LimelightHelpers.getBotPose3d_wpiBlue(frontLLName))
  //         : filterPose(
  //             LimelightHelpers.getBotPose3d_wpiRed(frontLLName)
  //                 .transformBy(Constants.field.RED_RIGHT_WALL));
  //   } else if (LimelightHelpers.getTV(backLLName)) {
  //     return filterPose(LimelightHelpers.getBotPose3d_wpiBlue(backLLName));
  //   } else return new Pose3d();
  // }

  public Pose3d getPose3d() {
    if(LimelightHelpers.getTV(name)) {
      return (DriverStation.getAlliance() == Alliance.Blue)
      ? LimelightHelpers.getBotPose3d_wpiBlue(name)
      : LimelightHelpers.getBotPose3d_wpiRed(name);
    } else {
      return new Pose3d();
    }
  }

  public boolean getTV() {
    return LimelightHelpers.getTV(name);
  }

  public double getLatency() {
    double time = Timer.getFPGATimestamp();
    if (LimelightHelpers.getTV(name)) {
      return time
          - (LimelightHelpers.getLatency_Capture(name) / 1000)
          - (LimelightHelpers.getLatency_Pipeline(name) / 1000);
    } else return 0.0;
  }
}
