// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  public static class controller {
    public static final int PILOT_PORT = 0;
    public static final double DEADBAND = 0.15;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static final class robot {
    public static final double A_LENGTH = Units.inchesToMeters(33);
    public static final double A_WIDTH = Units.inchesToMeters(27.5);
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS = 2048.0;
    public static final double NEO_ENCODER_TICKS = 42.0;

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

    public static final int SHOULDER_LEADER = 69;
    public static final int SHOULDER_FOLLOWER = 68;
    public static final int SHOULDER_ENCODER = 67;

    public static final int WRIST = 50;
    public static final int WRIST_ENCODER = 59;

    public static final int TELESCOPE = 39;
    public static final int TELESCOPE_LIMIT_SWITCH = 3;

    public static final int CLAW = 0;
    public static final int ULTRASONIC_SENSOR = 4;
  }

  public static class drivetrain {
    public static final double DRIVE_GEARING = 6.55; // 6.55 : 1
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * DRIVE_GEARING;
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_TICKS_PER_METER =
        DRIVE_TICKS_PER_ROTATION / WHEEL_CIRCUMFRENCE;
    public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;

    public static final double ANGLE_GEARING = 10.29; // 10.29 : 1
    public static final double ANGLE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * ANGLE_GEARING;
    public static final double ANGLE_TICKS_PER_DEGREE = ANGLE_TICKS_PER_ROTATION / 360.0;
    public static final double ANGLE_DEGREES_PER_TICK = 1.0 / ANGLE_TICKS_PER_DEGREE;

    public static final double MAX_VOLTS = 12.0;
    public static final double MAX_VELOCITY = 5.0;
    public static final double MAX_ACCEL = 12.0;
    public static final double MAX_CACCEL = 8.0;
    public static final double MAX_RADIANS = 3.0 * Math.PI;
    public static final double RAMP_RATE = 0.5;

    public static final Matrix<N3, N1> STATE_STD_DEVS =
        VecBuilder.fill(0.1, 0.1, 0.1); // [x, y, theta]
    public static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.01, 0.01, 0.01); // [x, y, theta]

    public static final Translation2d FL_LOCATION =
        new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    public static final Translation2d FR_LOCATION =
        new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    public static final Translation2d BL_LOCATION =
        new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    public static final Translation2d BR_LOCATION =
        new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));

    public static final double FL_ZERO = -138.427734375;
    public static final double BL_ZERO = -130.25390625;
    public static final double BR_ZERO = 34.27734375;
    public static final double FR_ZERO = -20.478515625;

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

    public static final class xyPID {
      public static final double kP = 3.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class rotPID {
      public static final double kP = 1.5;
      public static final double kI = 0.0;
      public static final double kD = 0.1;
    }

    public static final class correctionPID {
      public static final double kP = -0.1;
      public static final double kI = 0.0;
      public static final double kD = -0.006;
    }
  }

  public static final class sensors {
    public static final Translation3d FRONT_POSITION = new Translation3d(0, 0, 0);
    public static final Translation3d BACK_POSITION = new Translation3d(0, 0, 0);
  }

  public static final class claw {
    public static final double CUBE_DISTANCE = Units.inchesToMeters(9.313251);
    public static final double TOTAL_DISTANCE = Units.inchesToMeters(14.824533);

    public static final double SENSOR_SCALE = 2.0;
    public static final int SHOULDER_LEADER = 21;
    public static final int SHOULDER_FOLLOWER = 22;
    public static final int SHOULDER_ENCODER = 10;

    public static final int TELESCOPE_MOTOR = 23;
    public static final int WRIST = 24;
    public static final int WRIST_ENCODER = 9;
  }

  public static final class arm {
    public static final class shoulder {
      public static final double ZERO = 0.0;
      public static final double GEAR_RATIO = 132.741; // 101.1358 to 1
      public static final double TICKS_PER_ROTATION = robot.NEO_ENCODER_TICKS * GEAR_RATIO;
      public static final double TICKS_PER_DEGREE = TICKS_PER_ROTATION / 360;
      public static final double UPPER_LIMIT = 120;
      public static final double LOWER_LIMIT = -120;

      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kF = 0.0;

      public static final double MINVEL = 0.0;
      public static final double MAXVEL = 0.0;
      public static final double ALLOWERROR = 0.0;

      public static final double kS = 0.0;
      public static final double kG = 0.0;
      public static final double kV = 0.0;
    }

    public static final class telescope {

      public static final double ROTATIONS_PER_INCH = 62.197; // 62.197 rotations per inch
      public static final double TICKS_PER_INCH = ROTATIONS_PER_INCH * robot.NEO_ENCODER_TICKS;
      public static final double TICKS_PER_METER = Units.inchesToMeters(TICKS_PER_INCH);
      public static final double UPPER_LIMIT = 120;
      public static final double LOWER_LIMIT = -120;

      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kF = 0.0;

      public static final double kS = 0.0;
      public static final double kG = 0.0;
      public static final double kV = 0.0;

      public static final double MINVEL = 0.0;
      public static final double MAXVEL = 0.0;
      public static final double ALLOWERROR = 0.0;
    }

    public static final class wrist {
      public static final double GEAR_RATIO = 100; // 101.1358 to 1
      public static final double TICKS_PER_ROTATION = robot.NEO_ENCODER_TICKS * GEAR_RATIO;
      public static final double TICKS_PER_DEGREE = TICKS_PER_ROTATION / 360;

      public static final double ZERO = 0.0;
      public static final double UPPER_LIMIT = 120;
      public static final double LOWER_LIMIT = -120;

      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kF = 0.0;

      public static final double kS = 0.0;
      public static final double kG = 0.0;
      public static final double kV = 0.0;

      public static final double MINVEL = 0.0;
      public static final double MAXVEL = 0.0;
      public static final double ALLOWERROR = 0.0;
    }
  }
}
