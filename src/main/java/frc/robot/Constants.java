// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
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
    public static final double A_LENGTH = 0.59055; // Axel length (Meters).
    public static final double A_WIDTH = 0.48895; // Axel width (Meters).
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS = 2048.0; //Counts per revolution of the Falcon 500 motor.
    public static final double FALCON_MAX_TEMP = 50.0; //Max temperature of Falcon 500 (Celsius).
    public static final double FALCON_MAX_VEL = 6380.0;
}

public static class drivetrain {
    public static final double DRIVE_GEARING = 6.92; // Gear ratio of the drive motor.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //Diameter of the drive wheels (Meters).
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // Circumfrence of the drive wheels (Meters).
    public static final double DRIVE_ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE; // Rotations per meter of the drive wheels.
    public static final double DRIVE_COUNTS_PER_ROTATION = DRIVE_GEARING * robot.FALCON_ENCODER_TICKS; // Encoder counts per revolution of the drive wheel.
    public static final double DRIVE_COUNTS_PER_METER = DRIVE_ROTATIONS_PER_METER * DRIVE_COUNTS_PER_ROTATION; // Encoder ticks per meter of the drive wheels.

    public static final double ANGLE_GEARING = 11.57;
    public static final double ANGLE_TICKS_PER_DEGREE = (ANGLE_GEARING * robot.FALCON_ENCODER_TICKS) / 360.0;

    public static final double MAX_VOLTS = 12.0; // Maximum voltage allowed in the drivetrain.
    public static final double MAX_VELOCITY = 10.0; // Maximum velocity allowed in the drivetrain (Meters per Second).
    public static final double MAX_ACCEL = 20.0; // Maximum acceleration of the drivetrain in (Meters per Second Squared).
    public static final double MAX_CACCEL = 8.0; // Maximum centripital acceleration of the robot (Meters per Second Squared).
    public static final double MAX_RADIANS = 3.0 * Math.PI; // Maximum rotational velocity (Radians per Second).
    public static final double RAMP_RATE = 0.5;
    
    public static final double ROTATION_KV = 0.0;
    public static final double ROTATION_KA = 0.0;

    // Put together swerve module positions relative to the center of the robot.
    public static final Translation2d FrontLeftLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    public static final Translation2d FrontRightLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    public static final Translation2d BackLeftLocation = new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    public static final Translation2d BackRightLocation = new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));

    public static final class modules {
        public static final double M1_ZERO = 43.2421875;
        public static final double M2_ZERO = -121.9921875;
        public static final double M3_ZERO = -153.6328125;
        public static final double M4_ZERO = -50.09765625;
    }

    public static final class anglemotor {
        public static final double kP = -1e-2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class speedmotor {
        public static final double kP = 19e-2;
        public static final double kI = 0.0;
        public static final double kD = 1e-1;
    }

    public static final class xPID {
        public static final double kP = 5.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final PIDController xPID = new PIDController(kP, kI, kD);
    }
    public static final class yPID {
      public static final double kP = 5.2;
      public static final double kI = 0.0;
      public static final double kD = 0.0;  
      public static final PIDController yPID = new PIDController(kP, kI, kD);
  }

  public static final class thetaPID {
      public static final double kP = -17.25;
      public static final double kI = 0.0;
      public static final double kD = -0.05;  
      public static final ProfiledPIDController thetaPID = new ProfiledPIDController(kP, kI, kD, new Constraints(Math.PI * 2.0, Math.PI / 2.0));
  }

  
  }
}
  