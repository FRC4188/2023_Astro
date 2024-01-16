package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    interface Rev {
        boolean SIM_USE_ROBORIO_PID_FOR_POSITION = true;
    }

    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface Arm {
        Pose3d ROBOT_TO_ROOT_MOUNT_POSE = new Pose3d();

        double GEARING = 1;
        // TODO: find exact (this should be close)
        double MOVING_MASS_KG = Units.lbsToKilograms(30);
        double SPROCKET_RADIUS_M = Units.inchesToMeters(0.7955);
        double SPROCKET_CIRCUMFERENCE_M = 2 * Math.PI * SPROCKET_RADIUS_M;
        double MIN_TOTAL_EXT_M = 0;
        // TODO: find exact (this should be close)
        // TODO: this probably should be calculated from adding the stage one height and its extension height
        double MAX_TOTAL_EXT_M = Units.inchesToMeters(57.52);
        boolean SIMULATE_GRAVITY = true;
        double STAGE_ONE_HEIGHT = 0.83817;
        double STAGE_ONE_EXT_HEIGHT = Units.inchesToMeters(23.75);
        double STAGE_ONE_OFFSET = Units.inchesToMeters(0.5);
        double STAGE_TWO_HEIGHT = 0.20320;
        double STAGE_TWO_EXT_HEIGHT = Units.inchesToMeters(23.75);
        double STAGE_TWO_OFFSET = 0;
        // TODO: find exact?
        double EXT_MOI = 0.1937598419; // moi is in kg/m^2
    }
}
