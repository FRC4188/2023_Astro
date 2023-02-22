package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private static Arm instance;

  public static synchronized Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  private Telescope elevator = new Telescope();
  private Wrist wrist = new Wrist();
  private Shoulder shoulderJoint = new Shoulder();

  private Arm() {}

  public Pose2d getPosition() {
    double elevatorLength = elevator.getAngle();
    double shoulderAngle = Math.toRadians(shoulderJoint.getAngle());
    double wristAngle = Math.toRadians(wrist.getAngle());
    double x =
        elevatorLength * Math.sin(shoulderAngle)
            + Constants.claw.TOTAL_DISTANCE * Math.sin(shoulderAngle - wristAngle);
    double z =
        elevatorLength * Math.cos(shoulderAngle)
            + Constants.claw.TOTAL_DISTANCE * Math.cos(shoulderAngle - wristAngle);
    double pickUpAngle = Math.PI / 2 - shoulderAngle + wristAngle;

    return (new Pose2d(x, z, new Rotation2d(pickUpAngle)));
  }

  public double[] getInverseKinematic(Pose2d pose) {
    double x = pose.getX();
    double z = pose.getY();
    double pickUpAngle = pose.getRotation().getDegrees();
    double wristLength = Constants.claw.TOTAL_DISTANCE;

    double elevatorLength =
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

    return new double[] {elevatorLength, shoulderAngle, wristAngle};
  }

  public void setPosition(Pose2d pose) {
    double[] iKinematic = getInverseKinematic(pose);
    elevator.setAngle(iKinematic[0]);
    shoulderJoint.setAngle(iKinematic[1]);
    wrist.setAngle(iKinematic[2]);
  }
}
