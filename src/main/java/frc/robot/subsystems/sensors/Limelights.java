// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import csplib.utils.LimelightHelpers;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
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

  public Pose3d getPose3d() {
    if (LimelightHelpers.getTV(frontLLName)) {
      return LimelightHelpers.getBotPose3d_wpiBlue(frontLLName);
    } else if (LimelightHelpers.getTV(backLLName)) {
      return LimelightHelpers.getBotPose3d_wpiBlue(backLLName);
    } else return new Pose3d();
  }
}
