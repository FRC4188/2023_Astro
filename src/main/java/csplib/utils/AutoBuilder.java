// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package csplib.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class AutoBuilder {
  private static Drivetrain drivetrain = Drivetrain.getInstance();

  public static Command buildAuto(
      String pathName, HashMap<String, Command> eventMap, PathConstraints constraints) {
    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup(pathName, constraints);
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            drivetrain::getPose2d,
            drivetrain::resetOdometry,
            drivetrain.getKinematics(),
            drivetrain.getTransValues(),
            drivetrain.getRotValues(),
            drivetrain::setModuleStates,
            eventMap,
            true,
            drivetrain);
    return autoBuilder.fullAuto(paths);
  }

  public static Command buildAuto(String pathName, HashMap<String, Command> eventMap) {
    List<PathPlannerTrajectory> paths =
        PathPlanner.loadPathGroup(pathName, new PathConstraints(0, 0));
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            drivetrain::getPose2d,
            drivetrain::resetOdometry,
            drivetrain.getKinematics(),
            drivetrain.getTransValues(),
            drivetrain.getRotValues(),
            drivetrain::setModuleStates,
            eventMap,
            true,
            drivetrain);
    return autoBuilder.fullAuto(paths);
  }
}
