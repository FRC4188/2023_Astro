// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {
    double[] xyz = {0.0, 0.0, 1.0};
    Rotation3d rot = new Rotation3d(0.0, Math.PI / 2.0, 0.0);
    Translation3d gravity = new Translation3d(xyz[0], xyz[1], xyz[2]).rotateBy(rot.times(-1.0));
    rot = new Rotation3d(0.0, 0.0, 0.0);
    xyz[0] = 0.0;
    xyz[2] = 0.0;
    System.out.println(gravity.toString());
    System.out.println(new Translation3d(xyz[0], xyz[1], xyz[2]).rotateBy(rot.times(-1.0)).minus(gravity).rotateBy(rot));

  }


  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    //RobotBase.startRobot(Robot::new);
    new Main();
  }
}
