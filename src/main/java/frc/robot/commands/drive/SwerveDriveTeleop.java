package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.utils.teleop.ControllerUtils;

public class SwerveDriveTeleop extends Command {
    private final Swerve swerve;
    private final XboxController controller;

    public SwerveDriveTeleop(
            final Swerve swerve,
            final XboxController controller
    ) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        final double matchTime = DriverStation.getMatchTime();
        if (matchTime >= 0 && matchTime <= Constants.MATCH_END_THRESHOLD_SEC) {
            swerve.wheelXCommand().schedule();
            return;
        }

        final Translation2d leftStickSpeeds = ControllerUtils.getStickXYSquaredInput(
                controller.getLeftY(),
                controller.getLeftX(),
                0.01,
                Constants.Swerve.TELEOP_MAX_SPEED,
                1,
                1
        );

        final double rot = ControllerUtils.getStickSquaredInput(
                controller.getRightX(),
                0.01,
                Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED,
                1,
                1
        );

        swerve.drive(
                leftStickSpeeds.getX(),
                leftStickSpeeds.getY(),
                rot,
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
