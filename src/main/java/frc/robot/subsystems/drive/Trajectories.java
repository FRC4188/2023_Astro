package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.Constants;
import java.util.List;

 public final class Trajectories {
  private static final TrajectoryConfig autoConfig = Constants.drive.auto.CONFIG;
  private static final Transform2d transform =
      new Transform2d(new Pose2d(), new Pose2d());

  public static class simple {
    public static final Trajectory straight =
        TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(), new Pose2d(5.0, 0, new Rotation2d())), autoConfig)
            .transformBy(transform);
  }

  public static class twoball {
    public static final Trajectory toFirst =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0.87, 0.0, Rotation2d.fromDegrees(0.0))),
            autoConfig);
  }

  public static class fourball {
    public static final Trajectory toFirst =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0.87, 0.0, Rotation2d.fromDegrees(0.0))),
            autoConfig);

    public static final Trajectory toSecond =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.87, 0.0, Rotation2d.fromDegrees(92.81)),
                new Pose2d(0.69, 5.65, Rotation2d.fromDegrees(85.58))),
            autoConfig);

    public static final Trajectory toThird =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.69, 5.65, Rotation2d.fromDegrees(-106.2)),
                new Pose2d(-0.6, 3.0, Rotation2d.fromDegrees(-168.92))),
            autoConfig);
  }

  public static final class fiveball {
    public static final Trajectory first =
        TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(), new Pose2d(0.98, 0.0, Rotation2d.fromDegrees(0.0))), autoConfig);

    public static final Trajectory terminal1 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.98, 0.0, Rotation2d.fromDegrees(-96.0)),
                new Pose2d(0.86, -2.38, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(0.33, -6.05, Rotation2d.fromDegrees(-46.96))),
            autoConfig);

    public static final Trajectory terminal2 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.33, -6.05, Rotation2d.fromDegrees(130.0)),
                new Pose2d(-1.67, -4.18, Rotation2d.fromDegrees(-45.41 + 180.0))),
            new TrajectoryConfig(4.0, 1.0));

    public static final Trajectory shoot2 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-1.67, -4.18, Rotation2d.fromDegrees(54.18)),
                new Pose2d(-1, -3.15, Rotation2d.fromDegrees(54.18)),
                new Pose2d(-0.45, -3.0, Rotation2d.fromDegrees(53.45))),
            autoConfig);

    public static final Trajectory shoot3 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-1, -3.15, Rotation2d.fromDegrees(54.18)),
                new Pose2d(-0.26, -2.71, Rotation2d.fromDegrees(53.45))),
            autoConfig);
  }

  public static final class fiveballplus {
    public static final Trajectory first =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(0.0, -0.91, Rotation2d.fromDegrees(-90.0))),
            new TrajectoryConfig(3.0, 2.0)
                .addConstraint(new CentripetalAccelerationConstraint(2.0)));
    public static final Trajectory second =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, -0.91, Rotation2d.fromDegrees(90.0)),
                new Pose2d(-1.97, -0.1, Rotation2d.fromDegrees(-181.0)),
                new Pose2d(-2.25, 0.1, Rotation2d.fromDegrees(-180.0))),
            autoConfig);
    public static final Trajectory third =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-2.25, 0.1, Rotation2d.fromDegrees(-180.0)),
                new Pose2d(-6.25, -0.4, new Rotation2d(-2.45))),
            new TrajectoryConfig(1.5, 1.75));
    public static final Trajectory fourth =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-6.25, -0.4, Rotation2d.fromDegrees(45.0)),
                new Pose2d(-4.66, 0.38, new Rotation2d(-6.6))),
            new TrajectoryConfig(3.0, 2.0).setEndVelocity(1.0));

    public static final Trajectory fifth =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-4.66, 0.38, new Rotation2d(-6.6)),
                // new Pose2d(0.0, -0.83, new Rotation2d(0.0))
                new Pose2d(-2.69, 0.19, new Rotation2d(-Math.PI))),
            autoConfig.setStartVelocity(1.0));
  }

  public static final class sixball {
    public static final Trajectory first =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(0.0, -0.91, Rotation2d.fromDegrees(-90.0))),
            autoConfig);
    public static final Trajectory second =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, -0.91, Rotation2d.fromDegrees(90.0)),
                new Pose2d(-1.97, -0.1, Rotation2d.fromDegrees(-181.0))),
            autoConfig);
    public static final Trajectory third =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-1.97, -0.1, Rotation2d.fromDegrees(-181.0)),
                new Pose2d(-6.18, -0.68, new Rotation2d(-2.45))),
            autoConfig);
    public static final Trajectory fourth =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-6.18, -0.68, new Rotation2d(0.35)),
                new Pose2d(-4.33, 0.91, new Rotation2d(0.35))),
            new TrajectoryConfig(5.0, 1.75)
                .addConstraint(new CentripetalAccelerationConstraint(2.15))
                .setEndVelocity(2.0));
    public static final Trajectory fifth =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-4.33, 0.91, new Rotation2d(0.35)),
                new Pose2d(-3.09, 3.56, new Rotation2d(0.83))),
            autoConfig.setStartVelocity(2.0));
  }

  public static final class steal {
    public static final Trajectory first =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(6.728, 5.891, new Rotation2d(-0.911, 1.901)),
                new Pose2d(6.166, 7.049, new Rotation2d(-0.416, 0.787))),
            new TrajectoryConfig(2.0, 1.5)
                .addConstraint(new CentripetalAccelerationConstraint(1.5)));
    public static final Trajectory second =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(6.168, 6.969, new Rotation2d(-0.569, -1.669)),
                new Pose2d(7.439, 6.703, new Rotation2d(0.0, 1.043)),
                new Pose2d(7.439, 7.557, new Rotation2d(0.0, 1.669))),
            new TrajectoryConfig(2.0, 1.5)
                .addConstraint(new CentripetalAccelerationConstraint(1.5)));
  }

  public static final class hoard {
    public static final Trajectory first =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, 0.0, new Rotation2d(2.37)),
                new Pose2d(-0.79, 0.6, new Rotation2d(2.37))),
            new TrajectoryConfig(1.5, 2.0)
                .addConstraint(new CentripetalAccelerationConstraint(2.0)));
    public static final Trajectory second =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-0.79, 0.6, new Rotation2d(-1.81)),
                new Pose2d(-1.56, -1.48, new Rotation2d(-1.59))),
            new TrajectoryConfig(2.5, 1.5)
                .addConstraint(new CentripetalAccelerationConstraint(2.0)));
    public static final Trajectory third =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-1.56, -1.48, new Rotation2d(1.2)),
                new Pose2d(0.0, 1.76, new Rotation2d(1.42))),
            new TrajectoryConfig(2.5, 2.0)
                .addConstraint(new CentripetalAccelerationConstraint(2.0)));
    public static final Trajectory fourth =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, 1.76, new Rotation2d(-1.6)),
                new Pose2d(-3.42, 1.0, new Rotation2d(2.88))),
            new TrajectoryConfig(2.0, 2.0)
                .addConstraint(new CentripetalAccelerationConstraint(2.0)));
  }
}
