// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;

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

  private Arm() {}

  public void stow() {
    setPosition(0, Constants.arm.telescope.LOWER_LIMIT, Constants.arm.wrist.LOWER_LIMIT);
  }

  public void disable() {
    shoulder.disable();
    telescope.disable();
    wrist.disable();
  }

  public void set(double shoulderPercent, double telescopePercent, double wristPercent) {
    shoulder.set(shoulderPercent);
    telescope.set(telescopePercent);
    wrist.set(wristPercent);
  }

  public void setPosition(double shoulderAngle, double telescopeLength, double wristAngle) {
    shoulder.setAngle(shoulderAngle);
    telescope.setPosition(telescopeLength);
    wrist.setAngle(wristAngle);
  }

  public void setPosition(double[] positions) {
    shoulder.setAngle(positions[0]);
    telescope.setPosition(positions[1]);
    wrist.setAngle(positions[2]);
  }

  public void setToScore(double shoulderAngle, double telescopeLength, boolean isCube) {
    double cubeWristSet = (Math.abs(shoulderAngle) < 90) ? 90 - shoulderAngle : shoulderAngle - 90;
    double coneWristSet = 180 - shoulderAngle;
    double wristSet = (isCube) ? cubeWristSet : coneWristSet;
    setPosition(shoulderAngle, telescopeLength, wristSet);
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
