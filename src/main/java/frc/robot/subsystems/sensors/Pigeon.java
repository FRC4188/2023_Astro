package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends Pigeon2 {
  public Pigeon(int canID) {
    super(canID, "canivore");
    super.configFactoryDefault();
    super.clearStickyFaults();


    reset();
  }

  public void reset() {
    super.setYaw(0.0);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(((super.getYaw() + 180) % 360) - 180);
  }

  public double getRoll() {
    return super.getRoll();
  }

  public double getPitch() {
    return super.getPitch();
  }
}
