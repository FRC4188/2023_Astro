// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double FALCON_ENCODER_TICKS =
        2048.0; // Counts per revolution of the Falcon 500 motor.
    public static final double FALCON_MAX_VEL = 6380.0;

    public static final double NEO_ENCODER_TICKS = 42.0;

    public static final double MAX_TEMP = 50.0;
  }

  public static final class ids {
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
