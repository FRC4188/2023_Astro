package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator;

public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.REAL;
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.TESTING;
    double LOOP_PERIOD_SECONDS = 0.02;
    double MATCH_END_THRESHOLD_SEC = Units.millisecondsToSeconds(250);

    enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    enum CompetitionType {
        TESTING,
        COMPETITION
    }

    interface CTRE {
        double PHOENIX_5_100ms_PER_SECOND = 10;
        double PHOENIX_5_CANCODER_TICKS_PER_ROTATION = 4096d;
        // apply to CANCoder configuration which makes the sensor return in rotations
        double PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS = 1d / PHOENIX_5_CANCODER_TICKS_PER_ROTATION;
        String PHOENIX_5_CANCODER_UNIT_STRING_ROTS = "rots";

        boolean DISABLE_NEUTRAL_MODE_IN_SIM = true;
    }

    interface Swerve {
        double WHEEL_BASE = Units.inchesToMeters(33);
        double TRACK_WIDTH = Units.inchesToMeters(27.5);
        double ROBOT_MAX_SPEED = Units.feetToMeters(13.5); //TODO Was 10 which seems very wrong so please change for your module
        double ROBOT_MAX_ANGULAR_SPEED = 2 * Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_SPEED = 2;
        double TRAJECTORY_MAX_ACCELERATION = 2;
        double TRAJECTORY_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_ANGULAR_ACCELERATION = 1.5 * ROBOT_MAX_ANGULAR_SPEED;

        interface Modules {
            double WHEEL_RADIUS = 0.0508; //2 in
            double WHEEL_MASS = 0.2313321; //0.51 lbs //TODO Change this is from sds mk4i
            double DRIVE_WHEEL_MOMENT_OF_INERTIA = WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;
            double TURN_WHEEL_MOMENT_OF_INERTIA = 0.004; //TODO Change this is from sds mk4i
            double DRIVER_GEAR_RATIO = 5.46; //Assuming X2 Ratio Set
            double TURNER_GEAR_RATIO = 396 / 35.1;

            double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
            double MODULE_MAX_SPEED = Units.feetToMeters(14);
        }
    }

    interface NetworkTables {
        String AUTO_TABLE = "AutoSelector";
        String AUTO_PUBLISHER = "AutoOptions";
        String AUTO_SELECTED_SUBSCRIBER = "SelectedAuto";

        String PROFILE_TABLE = "ProfileSelector";
        String PROFILE_PUBLISHER = "ProfileOptions";
        String PROFILE_SELECTED_SUBSCRIBER = "SelectedProfile";

        boolean USE_STRUCT_AND_PROTOBUF = true;
    }

    interface Vision {
        PhotonPoseEstimator.PoseStrategy MULTI_TAG_POSE_STRATEGY =
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        //L = Left, R = Right, F = Forward, B = Backward (Facing)
        Transform3d ROBOT_TO_FR_APRILTAG_CAM_R = new Transform3d(
                new Translation3d(Units.inchesToMeters(13.449), Units.inchesToMeters(-13.762), Units.inchesToMeters(7.922)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-70))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(2.5));
        Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(5));
        double MULTI_TAG_MAX_AMBIGUITY = 0.3;
        double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    }
}
