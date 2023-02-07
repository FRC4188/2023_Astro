// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class robot {
    public static final double A_LENGTH = Units.inchesToMeters(33); // Axel length (Meters).
    public static final double A_WIDTH = Units.inchesToMeters(27.5); // Axel width (Meters).
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS =
        2048.0; // Counts per revolution of the Falcon 500 motor.
    public static final double FALCON_MAX_VEL = 6380.0;

    public static final double MAX_TEMP = 50.0;
  }

  public static final class ids {
    public static final int FL_SPEED = 1;
    public static final int FL_ANGLE = 2;
    public static final int FL_ENCODER = 11;

    public static final int BL_SPEED = 3;
    public static final int BL_ANGLE = 4;
    public static final int BL_ENCODER = 12;

    public static final int BR_SPEED = 5;
    public static final int BR_ANGLE = 6;
    public static final int BR_ENCODER = 13;

    public static final int FR_SPEED = 7;
    public static final int FR_ANGLE = 8;
    public static final int FR_ENCODER = 14;

    public static final int PIGEON = 15;
  }

  public static class drivetrain {
    public static final double DRIVE_GEARING = 6.55; // Gear ratio of the drive motor.
    public static final double WHEEL_DIAMETER =
        Units.inchesToMeters(4); // Diameter of the drive wheels (Meters).
    public static final double DRIVE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * DRIVE_GEARING;
    public static final double WHEEL_CIRCUMFRENCE =
        Math.PI * WHEEL_DIAMETER; // Circumfrence of the drive wheels (Meters).
    public static final double DRIVE_TICKS_PER_METER =
        DRIVE_TICKS_PER_ROTATION / WHEEL_CIRCUMFRENCE;
    public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;

    public static final double ANGLE_GEARING = 10.29; // 10.29 : 1
    public static final double ANGLE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * ANGLE_GEARING;
    public static final double ANGLE_TICKS_PER_DEGREE = ANGLE_TICKS_PER_ROTATION / 360.0;
    public static final double ANGLE_DEGREES_PER_TICK = 1.0 / ANGLE_TICKS_PER_DEGREE;

    public static final double MAX_VOLTS = 12.0; // Maximum voltage allowed in the drivetrain.
    public static final double MAX_VELOCITY =
        5.0; // Maximum velocity allowed in the drivetrain (Meters per Second).
    public static final double MAX_ACCEL =
        20.0; // Maximum acceleration of the drivetrain in (Meters per Second Squared).
    public static final double MAX_CACCEL =
        8.0; // Maximum centripital acceleration of the robot (Meters per Second Squared).
    public static final double MAX_RADIANS =
        3.0 * Math.PI; // Maximum rotational velocity (Radians per Second).
    public static final double RAMP_RATE = 0.5;

    public static final Matrix<N3, N1> STATE_STD_DEVS =
        VecBuilder.fill(0.1, 0.1, 0.1); // [x, y, theta]
    public static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.9, 0.9, 0.9); // [x, y, theta]

    public static final Translation2d FL_LOCATION =
        new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    public static final Translation2d FR_LOCATION =
        new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    public static final Translation2d BL_LOCATION =
        new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    public static final Translation2d BR_LOCATION =
        new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));

    public static final double FL_ZERO = -17.314453125;
    public static final double BL_ZERO = 29.970703125000004;
    public static final double BR_ZERO = 174.990234375;
    public static final double FR_ZERO = 175.341796875;

    public static final class angle {
      public static final double FL_kP = -0.008;
      public static final double FL_kI = 0.0;
      public static final double FL_kD = 0.0;

      public static final double BL_kP = -0.009;
      public static final double BL_kI = 0.0;
      public static final double BL_kD = 0.;

      public static final double BR_kP = -0.009;
      public static final double BR_kI = 0.0;
      public static final double BR_kD = 0.0;

      public static final double FR_kP = -0.008;
      public static final double FR_kI = 0.0;
      public static final double FR_kD = 0.0;
    }

    public static final class speed {
      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.02;
      public static final double kF = 0.05;
    }

    public static final class xPID {
      public static final double kP = 5.2; 
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class yPID {
      public static final double kP = 5.2;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class thetaPID {
      public static final double kP = -17.25;
      public static final double kI = 0.0;
      public static final double kD = -0.05;
    }
  }
}
