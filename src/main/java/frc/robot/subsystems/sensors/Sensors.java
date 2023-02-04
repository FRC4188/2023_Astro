package frc.robot.subsystems.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import aidenlib.control.*;
import aidenlib.*;
import aidenlib.math.*;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Pigeon pigeon = new Pigeon(Constants.ids.PIGEON);

  private Drivetrain drive = Drivetrain.getInstance();

  RotationEstimator3D rotEst = new RotationEstimator3D(Constants.StandardDevs.rotation);
  PoseEstimator3D poseEst = new PoseEstimator3D(() -> drive.getStdDevs());
  Pose3d pose = new Pose3d();

  /** Creates a new Sensors. */
  private Sensors() {
  }

  Translation3d prevAccel = new Translation3d();
  Translation3d prevVelEst = new Translation3d();
  Timer timer = new Timer();
  @Override
  public void periodic() {
    HashMap<String, LinkedList<Data>> vision = getVisionEstimates();

    Translation3d accel = getAccel();
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    double dt = timer.getDT();

    LinkedList<Data> xEstimates = new LinkedList<>();
    LinkedList<Data> yEstimates = new LinkedList<>();
    LinkedList<Data> zEstimates = new LinkedList<>();

    xEstimates.add(new Data(speeds.vxMetersPerSecond, Constants.StandardDevs.driveTrac));
    xEstimates.add(new Data(prevVelEst.getX() + Integral.linearSum(prevAccel.getX(), accel.getX(), dt), Constants.StandardDevs.accel));
    yEstimates.add(new Data(speeds.vyMetersPerSecond, Constants.StandardDevs.driveTrac));
    yEstimates.add(new Data(prevVelEst.getY() + Integral.linearSum(prevAccel.getY(), accel.getY(), dt), Constants.StandardDevs.accel));
    zEstimates.add(new Data(0.0, Constants.StandardDevs.driveTrac));
    zEstimates.add(new Data(prevVelEst.getZ() + Integral.linearSum(prevAccel.getZ(), accel.getZ(), dt), Constants.StandardDevs.accel));

    ArrayList<Data> velEstimate = WeightedFusion.calculateParameterized(List.of(xEstimates, yEstimates, zEstimates));

    prevAccel = accel;
    prevVelEst = new Translation3d(velEstimate.get(0).value, velEstimate.get(1).value, velEstimate.get(2).value);
    
    Rotation3d rot = rotEst.estimate(getRotation(), vision.get("r"), vision.get("p"), vision.get("y"));
    pose = poseEst.estimate(prevVelEst, rot, vision.get("X"), vision.get("Y"), vision.get("Z"));
  }

  public void updateDashboard() {
    Rotation3d rot = getRotation();
    SmartDashboard.putString("Rotation", String.format("Y: %f; P: %f; R: %f;",rot.getZ(), rot.getY(), rot.getX()));
    SmartDashboard.putString("Accel", getAccel().toString());
  }

  public double getPitch() {
    return pigeon.getRoll();
  }

  public double getRoll() {
    return pigeon.getPitch();
  }

  public void setPigeonAngle(Rotation3d angle) {
    pigeon.set(angle);
  }

  public Rotation3d getRotation() {
    return pigeon.get();
  }

  public Translation3d getAccel() {
    return pigeon.getAccel();
  }

  /**
   * ["X", "Y", "Z", "r", "p", "y"]
   */
  public HashMap<String, LinkedList<Data>> getVisionEstimates() {
    HashMap<String, LinkedList<Data>> map = new HashMap<String, LinkedList<Data>>();

    LinkedList<Data> X = new LinkedList<>();
    LinkedList<Data> Y = new LinkedList<>();
    LinkedList<Data> Z = new LinkedList<>();
    LinkedList<Data> r = new LinkedList<>();
    LinkedList<Data> p = new LinkedList<>();
    LinkedList<Data> y = new LinkedList<>();

    /*
     * Add vision measurements to the linked lists here.
     */

    map.put("X", X);
    map.put("Y", Y);
    map.put("Z", Z);
    map.put("r", r);
    map.put("p", p);
    map.put("y", y);
    return map;
  }

  public Pose3d getPose() {
    return pose;
  }

  public void setPose(Pose3d pose) {
    poseEst.setPose(pose);
    rotEst.setRotation(pose.getRotation());
  }

  public void setRotation(Rotation3d rot) {
    rotEst.setRotation(rot);
  }

  public void normalizeAccel() {
    pigeon.setG();
  }
}