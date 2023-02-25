// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
=======
import java.sql.Driver;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
=======
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
>>>>>>> 4cb66e0ea8021806465a430afb2d765de6919b02
=======
>>>>>>> 0691a41bf559ed70e1a05506f6ec6d5af5cde0b6
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

=======
>>>>>>> 56c362aebdce62bca6010cd79440815143a53295
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
<<<<<<< HEAD
<<<<<<< HEAD
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
  public static final class robot {
    public static final double A_LENGTH = 0.59055; // Axel length (Meters).
    public static final double A_WIDTH = 0.48895; // Axel width (Meters).
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS = 2048.0; //Counts per revolution of the Falcon 500 motor.
<<<<<<< HEAD
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
=======
    public static final double FALCON_MAX_VEL = 6380.0;
=======
=======
>>>>>>> 0691a41bf559ed70e1a05506f6ec6d5af5cde0b6
    public static class controller {
        public static final int PILOT_PORT = 0;
        public static final double DEADBAND = 0.15;
        public static final double TRIGGER_THRESHOLD = 0.6;
<<<<<<< HEAD
    }

    public static final class robot {
        public static final double A_LENGTH = Units.inchesToMeters(33); // Axel length (Meters).
        public static final double A_WIDTH = Units.inchesToMeters(27.5); // Axel width (Meters).
        public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

        public static final double FALCON_ENCODER_TICKS = 2048.0; //Counts per revolution of the Falcon 500 motor.
        public static final double FALCON_MAX_VEL = 6380.0;
>>>>>>> 4cb66e0ea8021806465a430afb2d765de6919b02

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
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //Diameter of the drive wheels (Meters).
    public static final double DRIVE_TICKS_PER_ROTATION = robot.FALCON_ENCODER_TICKS * DRIVE_GEARING;
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // Circumfrence of the drive wheels (Meters).
    public static final double DRIVE_TICKS_PER_METER = DRIVE_TICKS_PER_ROTATION / WHEEL_CIRCUMFRENCE;
    public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;

    public static final double ANGLE_GEARING = 10.29; // 10.29 : 1
    public static final double ANGLE_TICKS_PER_ROTATION = robot.FALCON_ENCODER_TICKS * ANGLE_GEARING;
    public static final double ANGLE_TICKS_PER_DEGREE = ANGLE_TICKS_PER_ROTATION / 360.0;
    public static final double ANGLE_DEGREES_PER_TICK = 1.0 / ANGLE_TICKS_PER_DEGREE;
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf

    public static final double MAX_VOLTS = 12.0; // Maximum voltage allowed in the drivetrain.
    public static final double MAX_VELOCITY = 1.0; // Maximum velocity allowed in the drivetrain (Meters per Second).
    public static final double MAX_ACCEL = 20.0; // Maximum acceleration of the drivetrain in (Meters per Second Squared).
    public static final double MAX_CACCEL = 8.0; // Maximum centripital acceleration of the robot (Meters per Second Squared).
    public static final double MAX_RADIANS = 3.0 * Math.PI; // Maximum rotational velocity (Radians per Second).
<<<<<<< HEAD
<<<<<<< HEAD

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
=======
    public static final double RAMP_RATE = 0.5;
    
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1); // [x, y, theta]
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.9, 0.9, 0.9); // [x, y, theta]

    public static final Translation2d FL_LOCATION = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    public static final Translation2d FR_LOCATION = new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
=======
    public static final double RAMP_RATE = 0.5;
    
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1); // [x, y, theta]
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.9, 0.9, 0.9); // [x, y, theta]

    public static final Translation2d FL_LOCATION = new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    public static final Translation2d FR_LOCATION = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
>>>>>>> 4cb66e0ea8021806465a430afb2d765de6919b02
    public static final Translation2d BL_LOCATION = new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    public static final Translation2d BR_LOCATION = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));

<<<<<<< HEAD
    public static final double M1_ZERO = 174.462890625;
    public static final double M2_ZERO = -19.16015625;
    public static final double M3_ZERO = 32.34375;
    public static final double M4_ZERO = -177.890625;

    public static final class angle {
        public static final double MAX_OMEGA =  360;
        public static final double MAX_ALPHA = 90;

        public static final double M1_kP = 5;
        public static final double M1_kI = 0.0;
        public static final double M1_kD = 1;
        public static final double M1_kS = 0.0;
        public static final double M1_kV = 0.0;

        public static final double M2_kP = 5;
        public static final double M2_kI = 0.0;
        public static final double M2_kD = 0.1;
        public static final double M2_kS = 0.0;
        public static final double M2_kV = 0.0;

        public static final double M3_kP = 5;
        public static final double M3_kI = 0.0;
        public static final double M3_kD = 0.3;
        public static final double M3_kS = 0.0;
        public static final double M3_kV = 0.0;

        public static final double M4_kP = 5;
        public static final double M4_kI = 0.0;
        public static final double M4_kD = 0.4;
        public static final double M4_kS = 0.0;
        public static final double M4_kV = 0.0;
    }

    public static final class speed {
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
        public static final double kP = 19e-2;
=======
    public static final double FL_ZERO = 175.693359375;
    public static final double BL_ZERO = -21.005859375000004;
    public static final double BR_ZERO = 27.685546875;
    public static final double FR_ZERO = 177.01171875000003;

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
>>>>>>> 4cb66e0ea8021806465a430afb2d765de6919b02
        public static final double kI = 0.0;
        public static final double kD = 0.02;
        public static final double kF = 0.05;
=======
    }

    public static final class robot {
        public static final double FALCON_ENCODER_TICKS = 2048.0;
        public static final double NEO_ENCODER_TICKS = 42.0;

        public static final double MAX_TEMP = 50.0; 
>>>>>>> 0691a41bf559ed70e1a05506f6ec6d5af5cde0b6
    }

    public static final class ids {
        public static final int SHOULDER_LEADER = 69;
        public static final int SHOULDER_FOLLOWER = 68;
        public static final int SHOULDER_ENCODER = 67;

        public static final int WRIST = 50;
        public static final int WRIST_ENCODER = 59;

        public static final int TELESCOPE = 39;
        public static final int TELESCOPE_LIMIT_SWITCH = 3;

    }

<<<<<<< HEAD
  public static final class thetaPID {
      public static final double kP = -17.25;
      public static final double kI = 0.0;
      public static final double kD = -0.05;  
      public static final ProfiledPIDController thetaPID = new ProfiledPIDController(kP, kI, kD, new Constraints(Math.PI * 2.0, Math.PI / 2.0));
  }
<<<<<<< HEAD

<<<<<<< HEAD
  
}

    public static class wrist
    {
        public static final double kS = 0.0; //idk yet
        public static final double kG = 0.0; //idk yet
        public static final double kV = 0.0; //idk yet
        public static final double kA = 0.0; //idk yet
        public static final double Kp = 0.0; //idk yet
        public static final double Ki = 0.0; //idk yet
        public static final double Kd = 0.0; //idk yet
        public static final TrapezoidProfile.Constraints constriants = new Constraints(0.0, 0.0); //idk yet
    }
  
}
=======

  
=======
>>>>>>> 4cb66e0ea8021806465a430afb2d765de6919b02
  }
}
  
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
=======
}
>>>>>>> 56c362aebdce62bca6010cd79440815143a53295
=======
    public static final class arm {
        public static final class shoulder {
            public static final double ZERO = 0.0;

            public static final double GEAR_RATIO = 132.741; // 132.741:1
            public static final double TICKS_PER_ROTATION = robot.NEO_ENCODER_TICKS * GEAR_RATIO;
            public static final double TICKS_PER_DEGREE = TICKS_PER_ROTATION / 360;

            public static final double UPPER_LIMIT = 120;
            public static final double LOWER_LIMIT = -120;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final Constraints CONSTRAINTS = new Constraints(0.0, 0.0);

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
        }

        public static class wrist {
            public static final double ZERO = 0.0;

            public static final double GEAR_RATIO = 100; // 100:1
            public static final double TICKS_PER_ROTATION = robot.NEO_ENCODER_TICKS * GEAR_RATIO;
            public static final double TICKS_PER_DEGREE = TICKS_PER_ROTATION / 360;

            public static final double UPPER_LIMIT = 120;
            public static final double LOWER_LIMIT = -120;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final Constraints CONSTRAINTS = new Constraints(0.0, 0.0);

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
        }

        public static class telescope {
            public static final double ROTATIONS_PER_INCH = 62.197; // 62.197 rotations per inch
            public static final double TICKS_PER_INCH = ROTATIONS_PER_INCH * robot.NEO_ENCODER_TICKS;
            public static final double TICKS_PER_METER = Units.inchesToMeters(TICKS_PER_INCH);

            public static final double UPPER_LIMIT = Units.inchesToMeters(51.125);
            public static final double LOWER_LIMIT = 0.0;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final Constraints CONSTRAINTS = new Constraints(0.0, 0.0);

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
        }
    }

    public static final class claw {
        public static final double CUBE_DISTANCE = Units.inchesToMeters(9.313251);
        public static final double TOTAL_DISTANCE = Units.inchesToMeters(14.824533);
    }
}
>>>>>>> 0691a41bf559ed70e1a05506f6ec6d5af5cde0b6
