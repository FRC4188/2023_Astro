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
    public static class controller {
        public static final int PILOT_PORT = 0;
        public static final double DEADBAND = 0.15;
        public static final double TRIGGER_THRESHOLD = 0.6;
    }

    public static final class robot {
        public static final double FALCON_ENCODER_TICKS = 2048.0; 

        public static final double MAX_TEMP = 50.0; 
    }

    public static final class ids {
        public static final int CLAW = 0;
        public static final int ULTRASONIC_SENSOR = 4;
    }

    public static final class claw {
        public static final double SENSOR_SCALE = 2.0;
    }
}
  