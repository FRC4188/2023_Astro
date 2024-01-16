package frc.robot.constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.SwerveModule;

import static frc.robot.constants.Constants.Swerve.TRACK_WIDTH;
import static frc.robot.constants.Constants.Swerve.WHEEL_BASE;

public class HardwareConstants {
    public record SwerveModuleConstants(
            String name,
            String moduleCANBus,
            Hardware hardware,
            Translation2d translationOffset,
            int driveMotorId,
            int turnMotorId,
            int turnEncoderId,
            double turnEncoderOffset
    ) {
        public enum Hardware {
            SwerveX_Falcon500_CANCoder
        }

        public static SwerveModule create(
                final SwerveModuleConstants swerveModuleConstants,
                final Constants.RobotMode currentMode
        ) {
            return switch (swerveModuleConstants.hardware) {
                case SwerveX_Falcon500_CANCoder -> SwerveModule.Builder.SwerveXFalcon500CANCoder(
                        swerveModuleConstants.name,
                        new TalonFX(swerveModuleConstants.driveMotorId, swerveModuleConstants.moduleCANBus),
                        new TalonFX(swerveModuleConstants.turnMotorId, swerveModuleConstants.moduleCANBus),
                        new CANcoder(swerveModuleConstants.turnEncoderId, swerveModuleConstants.moduleCANBus),
                        swerveModuleConstants.turnEncoderOffset,
                        currentMode
                );
            };
        }

        public SwerveModule create(final Constants.RobotMode currentMode) {
            return SwerveModuleConstants.create(this, currentMode);
        }
    }

    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(
            "FrontLeft",
            RobotMap.canivoreCANBus,
            SwerveModuleConstants.Hardware.SwerveX_Falcon500_CANCoder,
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            1,
            2,
            11,
            Units.degreesToRotations(-57.568359375)
    );

    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.canivoreCANBus,
            SwerveModuleConstants.Hardware.SwerveX_Falcon500_CANCoder,
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            7,
            8,
            14,
            Units.degreesToRotations(-75.498046875)
    );

    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.canivoreCANBus,
            SwerveModuleConstants.Hardware.SwerveX_Falcon500_CANCoder,
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            3,
            4,
            12,
            Units.degreesToRotations(-147.041015625)
    );

    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
            "BackRight",
            RobotMap.canivoreCANBus,
            SwerveModuleConstants.Hardware.SwerveX_Falcon500_CANCoder,
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            5,
            6,
            13,
            Units.degreesToRotations(43.505859375)
    );

    public record ArmConstants(
            String armCANBus,
            int shoulderLeaderMotorId,
            int shoulderFollowerMotorId,
            int shoulderEncoderId,
            int telescopeMotorId,
            int telescopeLimitSwitchDIOChannel,
            int wristMotorId
    ) {
    }

    public static final ArmConstants ARM = new ArmConstants(
            RobotMap.canivoreCANBus,
            21,
            22,
            9,
            17,
            9,
            24
    );

    public record ClawConstants(
            String clawCANBus,
            int wristMotorId,
            int rollerMotorId
    ) {
    }

    public static final ClawConstants CLAW = new ClawConstants(
            RobotMap.canivoreCANBus,
            24,
            16
    );
}
