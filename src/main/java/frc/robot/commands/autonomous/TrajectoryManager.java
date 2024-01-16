package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.utils.auto.AutoOption;
import frc.robot.wrappers.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

public class TrajectoryManager {
    public TrajectoryManager(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0),
                Constants.Swerve.Modules.MODULE_MAX_SPEED,
                Math.hypot(Constants.Swerve.WHEEL_BASE, Constants.Swerve.TRACK_WIDTH),
                new ReplanningConfig()
        );

        PathPlannerLogging.setLogCurrentPoseCallback(pose2d -> Logger.recordOutput("Auto/CurrentPose", pose2d));
        PathPlannerLogging.setLogTargetPoseCallback(pose2d -> {
            Logger.recordOutput("Auto/TargetPose", pose2d);
            Logger.recordOutput(
                    "Auto/DistanceToTarget",
                    swerve.getEstimatedPosition().minus(pose2d).getTranslation().getNorm()
            );
        });
        AutoBuilder.configureHolonomic(
                swerve::getEstimatedPosition,
                photonVision::resetPosition,
                swerve::getRobotRelativeSpeeds,
                swerve::drive,
                holonomicPathFollowerConfig,
                swerve
        );
    }

    public Command followAutoCommand(final AutoOption autoOption) {
        return new PathPlannerAuto(autoOption.name());
    }

    public static class FollowerContext {
        private final Swerve swerve;
        private final Arm arm;
        private final Claw claw;

        public FollowerContext(
                final Swerve swerve,
                final Arm arm,
                final Claw claw
        ) {
            this.swerve = swerve;
            this.arm = arm;
            this.claw = claw;
        }

        public Swerve getSwerve() {
            return swerve;
        }

        public Arm getArm() {
            return arm;
        }

        public Claw getClaw() {
            return claw;
        }
    }
}