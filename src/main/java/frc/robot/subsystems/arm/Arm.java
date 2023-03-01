package frc.robot.subsystems.arm;

import csplib.utils.Conversions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    SmartDashboard.putNumber("Shouler Angle", getShoulderAngle());
    SmartDashboard.putNumber("Shoulder Motor Angle", shoulder.getLeaderAngle());

    SmartDashboard.putNumber("Telescope Position", getTelescopeLength());
    SmartDashboard.putNumber("Telescope Current", telescope.getCurrent());
  }

  public void setPosition(Pose2d pose) {
    double[] iKinematic = getInverseKinematics(pose);
    telescope.setPosition(iKinematic[0]);
    shoulder.setAngle(iKinematic[1]);
    wrist.setAngle(iKinematic[2]);
  }

  public void stow() {
    shoulder.setAngle(0);
    telescope.zero();
    wrist.zero();
  }

  public void zeroWrist() {
    wrist.zero();
  }

  public void zeroTelescope() {
    telescope.zero();
  }

  public Pose2d getPosition() {
    double telescopeLength = telescope.getPosition();
    double shoulderAngle = Math.toRadians(shoulder.getAngle());
    double wristAngle = Math.toRadians(wrist.getAngle());
    double wristLength = 0;

    double x = telescopeLength * Math.sin(shoulderAngle) + wristLength * Math.sin(shoulderAngle - wristAngle);
    double z = telescopeLength * Math.cos(shoulderAngle) + wristLength * Math.cos(shoulderAngle - wristAngle);
    double pickUpAngle = Math.PI / 2 - shoulderAngle + wristAngle;

    Transform2d transform = new Transform2d(new Translation2d(0, Constants.robot.SHOULDER_HEIGHT), new Rotation2d());
    return new Pose2d(x, z, new Rotation2d(pickUpAngle)).transformBy(transform);
  }

  public void setTelescope(double percent) {
    telescope.set(percent);
  } 

  public void setShoulder(double percent) {
    shoulder.set(percent);
  }

  public void setWrist(double percent) {
    wrist.set(percent);
  }

  public void setShoulderPosition(double angle) {
    shoulder.setAngle(angle);
  }

  public void setTelescopePID(double kP, double kI, double kD, double kF) {
    telescope.setPID(kP, kI, kD, kF);
  }

  public void setShoulderPID(double kP, double kI, double kD, double kF) {
    shoulder.setPID(kP, kI, kD, kF);
  }

  public void setWristPID(double kP, double kI, double kD, double kF) {
    wrist.setPID(kP, kI, kD, kF);
  }

  public double[] getInverseKinematics(Pose2d pose) {
    double x = (pose.getX() > Constants.robot.MAX_EXTENSION) ? Constants.robot.MAX_EXTENSION : pose.getX();
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

    return new double[] {telescopeLength, Conversions.degreesUnsignedToSigned(shoulderAngle), Conversions.degreesUnsignedToSigned(wristAngle)};
  }

  public double getTelescopeLength() {
    return telescope.getPosition();
  }

  public double getShoulderAngle() {
    return shoulder.getAngle();
  }

  public double getWristAngle() {
    return wrist.getAngle();
  }
}
