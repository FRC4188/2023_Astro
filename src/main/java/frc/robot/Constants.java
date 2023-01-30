package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.sensors.Pigeon;

public class Constants {

  public static final class devices {
    public static final Pigeon pigeon = new Pigeon(15);
  }

  /** Constants for the entire robot as a whole. */
  public static final class robot {
    /** Meters */
    public static final double A_LENGTH = 0.714375;
    /** Meters */
    public static final double A_WIDTH = 0.574675;
    /** Meters */
    public final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    /** Counts / Revolution */
    public static final double FALCON_ENCODER_TICKS = 2048.0;
    /** Celsius */
    public static final double FALCON_MAX_TEMP = 50.0;
  }

  public static class drive {
    public static final double DRIVE_GEARING = 6.55; // Gear ratio of the drive motor.
    /** Meters */
    public static final double WHEEL_DIAMETER =
        Units.inchesToMeters(4); // Diameter of the drive wheels.
    /** Meters */
    public static final double WHEEL_CIRCUMFRENCE =
        Math.PI * WHEEL_DIAMETER; // Circumfrence of the drive wheels.
    /** Rotations / Meter */
    public static final double DRIVE_ROTATIONS_PER_METER =
        1.0 / WHEEL_CIRCUMFRENCE; // Rotations per meter of the drive wheels.
    /** Counts / Rotation */
    public static final double DRIVE_COUNTS_PER_ROTATION =
        DRIVE_GEARING
            * robot.FALCON_ENCODER_TICKS; // Encoder counts per revolution of the drive wheel.
    /** Counts / Meter */
    public static final double DRIVE_COUNTS_PER_METER =
        DRIVE_ROTATIONS_PER_METER
            * DRIVE_COUNTS_PER_ROTATION; // Encoder ticks per meter of the drive wheels.

    public static final double ANGLE_GEARING = 10.29;
    /** Counts / Degree */
    public static final double ANGLE_TICKS_PER_DEGREE =
        (ANGLE_GEARING * robot.FALCON_ENCODER_TICKS) / 360.0;

    /** Volts */
    public static final double MAX_VOLTS = 12.0; // Maximum voltage allowed in the drivetrain.
    /** Meters / Second */
    public static final double MAX_VELOCITY = 6.0; // Maximum velocity allowed in the drivetrain.
    /** Meters / Second^2 */
    public static final double MAX_ACCEL = 20.0; // Maximum acceleration of the drivetrain.
    /** Meters / Second^2 */
    public static final double MAX_CACCEL = 16.0; // Maximum centripital acceleration of the robot.
    /** Radians / Second */
    public static final double MAX_RADIANS = 2.0 * Math.PI; // Maximum rotational velocity.

    // Put together swerve module positions relative to the center of the robot.
    public static final Translation2d FrontLeftLocation =
        new Translation2d((Constants.robot.A_LENGTH / 2), (Constants.robot.A_WIDTH / 2));
    public static final Translation2d FrontRightLocation =
    new Translation2d((Constants.robot.A_LENGTH / 2), -(Constants.robot.A_WIDTH / 2));
    public static final Translation2d BackLeftLocation =
    new Translation2d(-(Constants.robot.A_LENGTH / 2), (Constants.robot.A_WIDTH / 2));
    public static final Translation2d BackRightLocation =
    new Translation2d(-(Constants.robot.A_LENGTH / 2), -(Constants.robot.A_WIDTH / 2));

    public static final double mod1zero = 6.064453125;
    public static final double mod2zero = -156.796875;
    public static final double mod3zero = 147.65625;
    public static final double mod4zero = -5.80078125;

    public static final class anglemotor {
      public static final double kP = -7e-3;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class speedmotor {
      public static final double kP = 17e-2;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class xPID {
      public static final double kP = 3.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0; // 0.052;
      public static final PIDController xPID = new PIDController(kP, kI, kD);
    }

    public static final class yPID {
      public static final double kP = 3.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0; // 0.052;
      public static final PIDController yPID = new PIDController(kP, kI, kD);
    }

    public static final class thetaPID {
      public static final double kP = -5.25; // -8.25;
      public static final double kI = 0.0;
      public static final double kD = 0.0; // -0.25;
      public static final ProfiledPIDController thetaPID =
          new ProfiledPIDController(kP, kI, kD, new Constraints(Math.PI * 2.0, Math.PI / 2.0));
    }

    public static final class auto {
      /** Meters / Second */
      public static final double MAX_VELOCITY = 1.0; // Maximum velocity allowed in the drivetrain.
      /** Meters / Second^2 */
      public static final double MAX_ACCEL = 1.75; // Maximum acceleration of the drivetrain.
      /** Meters / Second^2 */
      public static final double MAX_CACCEL =
          1.75; // Maximum centripital acceleration of the robot.

      public static final TrajectoryConfig CONFIG =
          new TrajectoryConfig(MAX_VELOCITY, MAX_ACCEL)
              .addConstraint(new CentripetalAccelerationConstraint(MAX_CACCEL));
    }
  }

  public static class StandardDevs {
    public static final double accel = 0.009065;
    public static final double rotation = Math.toRadians(0.4) / (60.0 * 50.0);
    public static final double driveTrac = 1.0;
  }

  public static class sensors {
    public static final Translation2d pigeonMounting = new Translation2d(0.0, 0.0);
  }
}
