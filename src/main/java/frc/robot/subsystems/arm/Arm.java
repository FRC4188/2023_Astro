// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;
import frc.robot.Constants.arm.wrist;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.sensors.Sensors;

/** Add your docs here. */
public class Arm {
  private static Arm instance;

  public static synchronized Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  private Shoulder shoulder = Shoulder.getInstance();
  private Telescope telescope = Telescope.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private Claw claw = Claw.getInstance();
  private Sensors sensors = Sensors.getInstance();

  private boolean isFlipped;

  private Arm() {}

  public void disable() {
    shoulder.disable();
    telescope.disable();
    wrist.disable();
  }

  public void setFlip(boolean isFlipped) {
    this.isFlipped = isFlipped;
  }

  public double[] getInverseKinematics(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double z = pose.getZ();
    double wristPAng = pose.getRotation().getAngle();
    double clawLen = claw.getClawLength();

    double radius = Math.hypot(x, y);

    double teleLen =
        Math.sqrt(
            Math.pow(radius, 2)
                + Math.pow(z, 2)
                + Math.pow(clawLen, 2)
                - 2 * radius * clawLen * Math.cos(wristPAng)
                - 2 * z * clawLen * Math.sin(wristPAng));
    double sAng =
        Math.PI / 2
            - Math.atan2(z - clawLen * Math.sin(wristPAng), x - clawLen * Math.cos(wristPAng));
    double wAngle =
        wristPAng
            - Math.atan2(z - clawLen * Math.sin(wristPAng), x - clawLen * Math.cos(wristPAng));
    double pigeonAngle = Math.atan2(y, x);

    return new double[] {sAng, teleLen, wAngle, pigeonAngle};
  }

  public boolean validate(double shoulderAngle, double telescopeLength, double wristAngle) {
    boolean shoulderGood = Math.abs(shoulderAngle - shoulder.getAngle()) < 1;
    boolean telescopeGood = Math.abs(telescopeLength - telescope.getPosition()) < 0.1;
    boolean wristGood = Math.abs(wristAngle - wrist.getAngle()) < 1;

    return shoulderGood && telescopeGood && wristGood;
  }

  public boolean validate(double[] positions) {
    boolean shoulderGood = Math.abs(positions[0] - shoulder.getAngle()) < 1;
    boolean telescopeGood = Math.abs(positions[1] - telescope.getPosition()) < 0.1;
    boolean wristGood = Math.abs(positions[2] - wrist.getAngle()) < 1;

    return shoulderGood && telescopeGood && wristGood;
  }

  public Pose3d getPosition() {
    double sAng = Math.toRadians(shoulder.getAngle());
    double teleLen = telescope.getPosition();
    double wAng = Math.toRadians(wrist.getAngle());
    double clawLen = claw.getClawLength();

    double wristPAngle = Math.PI - sAng - wAng;
    double x =
        (teleLen * Math.sin(sAng) + clawLen * Math.sin(sAng - wAng))
            * Math.cos(sensors.getRotation2d().getRadians());
    double y =
        (teleLen * Math.sin(sAng) + clawLen * Math.sin(sAng - wAng))
            * Math.sin(sensors.getRotation2d().getRadians());
    double z = (teleLen * Math.cos(sAng) + clawLen * Math.cos(sAng - wAng));

    return new Pose3d(x, y, z, new Rotation3d(0, wristPAngle, 0))
        .relativeTo(new Pose3d(0, 0, Constants.robot.SHOULDER_HEIGHT, new Rotation3d()));
  }

  public boolean getIsFlipped() {
    return isFlipped;
  }

  public Shoulder getShoulder() {
    return shoulder;
  }

  public Telescope getTelescope() {
    return telescope;
  }

  public Wrist getWrist() {
    return wrist;
  }
}
