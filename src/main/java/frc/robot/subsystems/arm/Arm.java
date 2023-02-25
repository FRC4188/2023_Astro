package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private static Arm instance;

  public static synchronized Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  private Telescope telescope = new Telescope();
  private Wrist wrist = new Wrist();
  private Shoulder shoulder = new Shoulder();

  private Arm() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
    SmartDashboard.putNumber("Telescope Position", getTelescopeLength());
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
  }

  public void setPosition(Pose2d pose) {
    double[] iKinematic = getInverseKinematic(pose);
    telescope.setPosition(iKinematic[0]);
    shoulder.setAngle(iKinematic[1]);
    wrist.setAngle(iKinematic[2]);
  }

  public void stow() {
    shoulder.setAngle(0);
    telescope.zero();
    wrist.setAngle(Constants.arm.wrist.LOWER_LIMIT);
  }

  public Pose2d getPosition() {
    double telescopeLength = telescope.getPosition();
    double shoulderAngle = Math.toRadians(shoulder.getAngle());
    double wristAngle = Math.toRadians(wrist.getAngle());
    double x = telescopeLength * Math.sin(shoulderAngle) + 0 * Math.sin(shoulderAngle - wristAngle);
    double z = telescopeLength * Math.cos(shoulderAngle) + 0 * Math.cos(shoulderAngle - wristAngle);
    double pickUpAngle = Math.PI / 2 - shoulderAngle + wristAngle;

    return (new Pose2d(x, z, new Rotation2d(pickUpAngle)));
  }

  public double[] getInverseKinematic(Pose2d pose) {
    double x = pose.getX();
    double z = pose.getY();
    double pickUpAngle = pose.getRotation().getDegrees();
    double wristLength = 0;

    double telescopeLength =
        Math.sqrt(
            Math.pow(x, 2)
                + Math.pow(z, 2)
                + Math.pow(wristLength, 2)
                - 2 * x * wristLength * Math.cos(pickUpAngle)
                - 2 * z * wristLength * Math.sin(pickUpAngle));
    double shoulderAngle =
        90
            - Math.toDegrees(
                Math.atan2(
                    (z - wristLength * Math.sin(pickUpAngle)),
                    (x - wristLength * Math.cos(pickUpAngle))));
    double wristAngle =
        0
            - Math.toDegrees(
                Math.atan2(
                    (z - wristLength * Math.sin(pickUpAngle)),
                    (x - wristLength * Math.cos(pickUpAngle))));

    return new double[] {telescopeLength, shoulderAngle, wristAngle};
  }

  private double getTelescopeLength() {
    return telescope.getPosition();
  }

  private double getShoulderAngle() {
    return shoulder.getAngle();
  }

  private double getWristAngle() {
    return wrist.getAngle();
  }
}
