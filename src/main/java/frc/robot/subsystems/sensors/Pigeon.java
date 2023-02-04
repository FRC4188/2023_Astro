package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Pigeon extends Pigeon2 {

  private double[] offsets = {0.0, 0.0, 0.0};

  private Translation3d gravity = new Translation3d();

  public Pigeon(int canID) {
    super(canID);

    super.configFactoryDefault();
    set(new Rotation3d());

    super.clearStickyFaults();
  }

  public Rotation3d getOmegaRadians() {
    double[] dps = {0.0, 0.0, 0.0};
    super.getRawGyro(dps);

    return new Rotation3d(Math.toRadians(dps[1]), Math.toRadians(dps[0]), Math.toRadians(dps[2]));
  }

  public Rotation3d get() {
    return new Rotation3d(Math.toRadians(super.getRoll()) + offsets[0], Math.toRadians(super.getPitch()) + offsets[1], Math.toRadians(super.getYaw()) + offsets[2]);
  }

  public Rotation2d getCompass() {
    return Rotation2d.fromDegrees((super.getCompassHeading() + 180.0) % 360.0 - 180.0);
  }

  public void set(Rotation3d angle) {
    Rotation3d current = get();
    offsets[0] += angle.getY() - current.getY();
    offsets[1] += angle.getX() - current.getX();
    offsets[2] += angle.getZ() - current.getZ();
  }

  public Translation3d getAccel() {
    double[] xyz = {0.0, 0.0, 0.0};
    super.getGravityVector(xyz);
    Rotation3d rot = get();
    return new Translation3d(xyz[0], xyz[1], xyz[2]).rotateBy(rot.times(-1.0)).minus(gravity).rotateBy(rot).times(9.8);
  }

  public void setG() {
    double[] xyz = {0.0, 0.0, 0.0};
    super.getGravityVector(xyz);
    gravity = new Translation3d(xyz[0], xyz[1], xyz[2]).rotateBy(get().times(-1.0));
  }
}