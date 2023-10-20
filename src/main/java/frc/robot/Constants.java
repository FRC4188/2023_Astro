// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static class controller {
    public static final int PILOT_PORT = 0;
    public static final int COPILOT_PORT = 1;
    public static final double DEADBAND = 0.15;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static final class field {
    public static double GRID_TOP_X = 0.4;
    public static double GRID_TOP_Z = Units.inchesToMeters(46.0);

    public static double GRID_MID_X = 0.8;
    public static double GRID_MID_Z = Units.inchesToMeters(34.0);

    public static double GRID_BOTTOM_X = 1.2;
    public static double GRID_BOTTOM_Z = Units.inchesToMeters(5.0);

    public static double DOUBLE_Z = 0.0;
    public static double SINGLE_Z = 0.0;

    public static double FIELD_WIDTH = Units.feetToMeters(26.2916);

    public static Transform3d RED_RIGHT_WALL =
        new Transform3d(
            new Translation3d(FIELD_WIDTH, new Rotation3d(0, 0, Math.PI)),
            new Rotation3d(0, 0, Math.PI));
  }

  public static final class robot {
    public static final double A_LENGTH = Units.inchesToMeters(33);
    public static final double A_WIDTH = Units.inchesToMeters(27.5);
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double MAX_EXTENSION = Units.inchesToMeters(48);

    public static final double FALCON_ENCODER_TICKS = 2048.0;
    public static final double NEO_ENCODER_TICKS = 42.0;

    public static final double MAX_TEMP = 50.0;

    public static final double SHOULDER_HEIGHT = Units.inchesToMeters(20);

    public static final double CUBE_DISTANCE = Units.inchesToMeters(9.313251);
    public static final double TOTAL_DISTANCE = Units.inchesToMeters(14.824533);
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

    public static final int SHOULDER_LEADER = 21;
    public static final int SHOULDER_FOLLOWER = 22;
    public static final int SHOULDER_ENCODER = 9;

    public static final int WRIST = 24;
    public static final int WRIST_ENCODER = 10;

    public static final int TELESCOPE = 17;
    public static final int TELESCOPE_LIMIT_SWITCH = 9;

    public static final int CLAW = 16;
    public static final int ULTRASONIC_SENSOR = 0;
  }

  public static class drivetrain {
    public static final double DRIVE_GEARING = 5.50; // 5.50 : 1
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
    public static final double MAX_VELOCITY = 10.0; // ?
    public static final double MAX_ACCEL = 7.0;
    public static final double MAX_CACCEL = 8.0;
    public static final double MAX_RADIANS = 5 * Math.PI;
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

    public static final double FL_ZERO = -57.568359375;
    public static final double BL_ZERO = 169.892578125;
    public static final double BR_ZERO = 43.505859375;
    public static final double FR_ZERO = -75.498046875;

    public static final class angle {
      public static final double FL_kP = -0.008;
      public static final double FL_kI = 0.0;
      public static final double FL_kD = 0.0;

      public static final double BL_kP = -0.009;
      public static final double BL_kI = 0.0;
      public static final double BL_kD = 0.0;

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
      public static final double kP = 2.5;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class correctionPID {
      public static final double kP = -0.1;
      public static final double kI = 0.0;
      public static final double kD = -0.006;
    }

    public static final class balancePID {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }

  public static final class sensors {
    public static final String LEFT_NAME = "limelight-left";
    public static final String RIGHT_NAME = "limelight-right";

    public static final Translation3d LEFT_POSITION = new Translation3d(0, 0, 0);
    public static final Translation3d RIGHT_POSITION = new Translation3d(0, 0, 0);

    public static Rotation3d LEFT_ROTATION = new Rotation3d(0, 0, 0);
    public static Rotation3d RIGHT_ROTATION = new Rotation3d(0, 0, 0);
  }

  public static final class claw {
    public static final double CUBE_TRIGGER = 0.009;
    public static final double SENSOR_SCALE = 1.984;
  }

  public static final class arm {
    public static final class configs {
      // double[] = {Shoulder Angle, Telescope Length, Wrist Angle}
      // positive wrist angle = energy chain side
      public static final double[][] HIGH_CONE = {{44.0, 1.22, 87.0}, {-40.0, 1.22, -92.0}};
      public static final double[][] HIGH_CUBE = {{55.0, 1.25, 120}, {-51.0, 1.15, -20.0}};

      public static final double[][] MID_CONE = {{42.0, 0.76, 100.0}, {-40.0, 0.76, -100.0}};
      public static final double[][] MID_CUBE = {{55.0, 0.5, 120}, {-51.0, 0.5, -20.0}};

      public static final double[][] LOW_CONE = {{60.0, 0.1875, 100.0}, {-58.0, 0.1875, -100.0}};
      public static final double[][] LOW_CUBE = {{75.0, 0.1875, 100.0}, {-73.0, 0.1875, -20.0}};

      public static final double[][] SS_CONE = {{90.0, 0.1875, -30.0}, {-88.0, 0.1875, 30.0}};
      public static final double[][] SS_CUBE = {{49.0, 0.1875, 100.0}, {-84.0, 0.1875, 112.0}};

      public static final double[][] DS_CONE = {{14.0, 0.1875, 82.0}, {-12.0, 0.1875, -82.0}};
      public static final double[][] DS_CUBE = {{0, 0.1875, 0}, {0, 0.1875, 0}};

      public static final double[][] FLOOR_CONE = {{83.0, 0.1875, 57.0}, {-73.0, 0.1875, -57.0}};
      public static final double[][] FLOOR_CUBE = {{104.0, 0.45, 125.0}, {-111.0, 0.25, 35.0}};
      public static final double[][] TIPPED_CONE = {{117, 0.1875, 7.0}, {-115, 0.1875, 5.0}};
      public static final double[][] BACK_TIPPED_CONE = {{86.0, 0.5, 125.0}, {-85.0, 0.5, -125.0}};

      public static final double[][] YOSHI_CUBE = {{90, 0.3, -10}, {90, 0.5, 90}};

      public static final double[] RESET = {0, 0.1875, 110};
    }

    public static final class shoulder {
      public static final double ZERO = 62.75390625;
      public static final double GEAR_RATIO = 132.741; // 132.741 to 1
      public static final double ROTATIONS_PER_DEGREE = GEAR_RATIO / 360;

      // public static final double UPPER_LIMIT = 98.0;
      // public static final double LOWER_LIMIT = -107.0;

      public static final double UPPER_LIMIT = 117.0;
      public static final double LOWER_LIMIT = -117.0;

      public static final double kP = 0.02;
      public static final double kI = 0.0;
      public static final double kD = 0.00;
      public static final double kF = 0.0;

      public static final double MAX_VEL = 1000.0;
      public static final double MAX_ACCEL = 720.0;

      public static final double ALLOWED_ERROR = 0.75;

      public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);

      public static final double kS = 0.0;
      public static final double kG = 0.0;
      public static final double kV = 0.0;
    }

    public static final class telescope {
      public static final double TICKS_PER_INCH = 2048; // 57.014 rotations per inch
      public static final double TICKS_PER_METER = 2048 / 0.025;

      public static final double UPPER_LIMIT = Units.metersToInches(1.4) * 2048;
      public static final double LOWER_LIMIT = 7.5 * 2048;

      public static final double kP = 60.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kF = 0.0;

      public static final double kS = 0.4495;
      public static final double kG = 1.21;
      public static final double kV = 3.8186;

      public static final double MAX_VEL = 40.0;
      public static final double MAX_ACCEL = 35.0;

      public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);
      public static final double ALLOWED_ERROR = 0.05;
    }

    public static final class wrist {
      public static final double ZERO = 0;

      public static final double GEAR_RATIO = 100; // 100 to 1
      public static final double ROTATIONS_PER_DEGREE = GEAR_RATIO / 360;

      public static final double UPPER_LIMIT = 126.4748;
      public static final double LOWER_LIMIT = -126.4748;

      public static final double kP = 0.03;
      public static final double kI = 0.0;
      public static final double kD = 0.000;
      public static final double kF = 0.000;

      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;

      public static final double MAX_VEL = 1000.0;
      public static final double MAX_ACCEL = 520.0;
      public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);
      public static final double ALLOWED_ERROR = 1;
    }
  }
}
