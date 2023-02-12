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
import frc.robot.AidenLib.Timer;
import frc.robot.AidenLib.control.PoseEstimator3D;
import frc.robot.AidenLib.control.RotationEstimator3D;
import frc.robot.AidenLib.math.Data;
import frc.robot.AidenLib.math.Integral;
import frc.robot.AidenLib.math.WeightedFusion;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Pigeon pigeon = new Pigeon(15);


  /** Creates a new Sensors. */
  private Sensors() {
  }

  Translation3d prevAccel = new Translation3d();
  Translation3d prevVelEst = new Translation3d();
  Timer timer = new Timer();
  @Override
  public void periodic() {
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

  public void normalizeAccel() {
    pigeon.setG();
  }
}
