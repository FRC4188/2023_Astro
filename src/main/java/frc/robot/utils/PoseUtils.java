package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;

public class PoseUtils {
    public static final double EPSILON = 1E-9;

    private PoseUtils() {}

    public enum MirroringBehavior {
        MIRROR_ACROSS_GRID_CENTER_POINT,
        MIRROR_ACROSS_X_CENTER
    }

    private static DriverStation.Alliance getOptionalOrElseBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public static Pose2d flipPose(final Pose2d poseToFlip) {
        return flipPoseWithRelative(poseToFlip, FieldConstants.FLIPPING_POSE);
    }

    public static Pose2d flipPoseWithRelative(final Pose2d poseToFlip, final Pose2d relative) {
        return poseToFlip.relativeTo(relative);
    }

    public static Pose2d flipPoseWithRelativeAndOffset(
            final Pose2d poseToFlip,
            final Pose2d relative,
            final Translation2d offset
    ) {
        final Pose2d flippedPose;
        return new Pose2d(
                (flippedPose = flipPoseWithRelative(poseToFlip, relative)).getX() + offset.getX(),
                flippedPose.getY() + offset.getY(),
                flippedPose.getRotation()
        );
    }

    public static Translation2d flipTranslation(final Translation2d translationToFlip) {
        return flipPose(new Pose2d(translationToFlip, Rotation2d.fromDegrees(0))).getTranslation();
    }

    public static Translation2d flipTranslationToBlueAllianceByOrigin(final Translation2d translationToFlip) {
        return PoseUtils.getOptionalOrElseBlueAlliance() == DriverStation.Alliance.Red
                ? flipTranslation(translationToFlip)
                : translationToFlip;
    }

    public static Translation2d localizeTranslationOnAlliance(
            final Translation2d originalTranslation,
            final DriverStation.Alliance sourceAlliance,
            final MirroringBehavior mirroringBehavior
    ) {
        final DriverStation.Alliance alliance = PoseUtils.getOptionalOrElseBlueAlliance();
        if (sourceAlliance == alliance) {
            return originalTranslation;
        } else if (alliance == DriverStation.Alliance.Red && sourceAlliance == DriverStation.Alliance.Blue) {
            return switch(mirroringBehavior) {
                case MIRROR_ACROSS_GRID_CENTER_POINT -> new Translation2d(
                        originalTranslation.getX(),
                        originalTranslation.getY() + FieldConstants.LOADING_ZONE_WIDTH_Y_METERS
                );
                case MIRROR_ACROSS_X_CENTER -> flipTranslationToBlueAllianceByOrigin(mirrorTranslationToAlliance(
                        originalTranslation, sourceAlliance, alliance
                ));
            };
        } else if (alliance == DriverStation.Alliance.Blue && sourceAlliance == DriverStation.Alliance.Red) {
            return new Translation2d(
                    originalTranslation.getX(),
                    originalTranslation.getY() - FieldConstants.LOADING_ZONE_WIDTH_Y_METERS
            );
        } else {
            throw new UnsupportedOperationException("not yet implemented!");
        }
    }

    public static Pose2d localizePoseOnAlliance(
            final Pose2d originalPose,
            final DriverStation.Alliance sourceAlliance,
            final MirroringBehavior mirroringBehavior
    ) {
        final DriverStation.Alliance alliance = PoseUtils.getOptionalOrElseBlueAlliance();
        if (sourceAlliance == alliance) {
            return originalPose;
        } else if (alliance == DriverStation.Alliance.Red && sourceAlliance == DriverStation.Alliance.Blue) {
            return new Pose2d(
                    localizeTranslationOnAlliance(originalPose.getTranslation(), sourceAlliance, mirroringBehavior),
                    originalPose.getRotation()
            );
        } else if (alliance == DriverStation.Alliance.Blue && sourceAlliance == DriverStation.Alliance.Red) {
            throw new UnsupportedOperationException("not yet implemented!");
        } else {
            throw new UnsupportedOperationException("not yet implemented!");
        }
    }

    public static Translation2d mirrorTranslationToAlliance(
            final Translation2d originalTranslation,
            final DriverStation.Alliance sourceAlliance,
            final DriverStation.Alliance desiredAlliance
    ) {
        if (sourceAlliance == desiredAlliance) {
            return originalTranslation;
        } else if (sourceAlliance == DriverStation.Alliance.Blue && desiredAlliance == DriverStation.Alliance.Red) {
            return new Translation2d(
                    FieldConstants.FIELD_LENGTH_X_METERS - originalTranslation.getX(),
                    originalTranslation.getY()
            );
        } else if (sourceAlliance == DriverStation.Alliance.Red && desiredAlliance == DriverStation.Alliance.Blue) {
            throw new UnsupportedOperationException("not yet implemented!");
        } else {
            throw new UnsupportedOperationException("not yet implemented!");
        }
    }

    public static Pose2d mirrorPoseToAlliance(
            final Pose2d originalPose,
            final DriverStation.Alliance sourceAlliance,
            final DriverStation.Alliance desiredAlliance
    ) {
        if (sourceAlliance == desiredAlliance) {
            return originalPose;
        } else if (sourceAlliance == DriverStation.Alliance.Blue && desiredAlliance == DriverStation.Alliance.Red) {
            return new Pose2d(
                    mirrorTranslationToAlliance(originalPose.getTranslation(), sourceAlliance, desiredAlliance),
                    originalPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
            );
        } else if (sourceAlliance == DriverStation.Alliance.Red && desiredAlliance == DriverStation.Alliance.Blue) {
            throw new UnsupportedOperationException("not yet implemented!");
        } else {
            throw new UnsupportedOperationException("not yet implemented!");
        }
    }

    public static boolean isInField(final Pose3d pose3d) {
        return pose3d.getX() >= -FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getX() <= FieldConstants.FIELD_LENGTH_X_METERS + FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getY() >= -FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getY() <= FieldConstants.FIELD_WIDTH_Y_METERS + FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getZ() >= -FieldConstants.FIELD_WITHIN_Z_MARGIN
                && pose3d.getZ() <= FieldConstants.FIELD_WITHIN_Z_MARGIN;
    }

    public enum Axis {
        X(new Transform3d(new Translation3d(1, 0, 0), new Rotation3d())),
        Y(new Transform3d(new Translation3d(0, 1, 0), new Rotation3d())),
        Z(new Transform3d(new Translation3d(0, 0, 1), new Rotation3d())),
        Roll(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(1, 0, 0))),
        Pitch(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 1, 0))),
        Yaw(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 1)));

        private final Transform3d axisUnitTransform;

        Axis(final Transform3d axisUnitTransform) {
            this.axisUnitTransform = axisUnitTransform;
        }

        public Transform3d getWithScalarOffset(final double scalarOffset) {
            return axisUnitTransform.times(scalarOffset);
        }
    }

    public static Pose3d withAxisOffset(final Pose3d pose3d, final Axis axis, final double offset) {
        return pose3d.plus(axis.getWithScalarOffset(offset));
    }

    public static Pose2d flipPose2dByOriginPosition(
            final Pose2d pose2d, final AprilTagFieldLayout.OriginPosition originPosition
    ) {
        return originPosition == AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
                ? pose2d
                : PoseUtils.flipPose(pose2d);
    }
}
