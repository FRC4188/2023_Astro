// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package csplib.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoConfigs;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class AutoBuilder {
  private static Drivetrain drivetrain = Drivetrain.getInstance();

  public static Command buildAuto(String pathName, HashMap<String, Command> eventMap, PathConstraints... constraints) {
    PathConstraints[] others = Arrays.copyOfRange(constraints, 1, constraints.length);
    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup(pathName, constraints[0], others);
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            drivetrain::getPose2d,
            drivetrain::resetOdometry,
            drivetrain.getKinematics(),
            drivetrain.getTransValues(),
            drivetrain.getRotValues(),
            drivetrain::setModuleStates,
            eventMap,
            false,
            drivetrain);
    try {
      return autoBuilder.fullAuto(paths);
    } catch (Exception e) {
      // TODO: handle exception
      DriverStation.reportError(e.getMessage(), false);
      return new SequentialCommandGroup();
    }
  }

  public static Command buildAuto(String pathName, HashMap<String, Command> eventMap) {
    return buildAuto(pathName, eventMap, AutoConfigs.DEFAULT_CONSTRAINTS);
  }
}
