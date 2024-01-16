package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.ArmClawTeleop;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.utils.SuperstructureStates;

import java.util.List;

public class ButtonBindings {
    public static void bindAll(final RobotContainer robotContainer) {
        final CommandXboxController driverController = robotContainer.pilot;
        final CommandXboxController coDriverController = robotContainer.copilot;

        final Swerve swerve = robotContainer.swerve;
        final Arm arm = robotContainer.arm;
        final Claw claw = robotContainer.claw;

        driverController.a().onTrue(Commands.runOnce(() -> swerve.zeroRotation(robotContainer.photonVision)));

        coDriverController.leftBumper().debounce(0.05).onTrue(Commands.runOnce(() -> {
            claw.setGamePiece(SuperstructureStates.ClawGamePiece.CONE);
        }));

        coDriverController.rightBumper().debounce(0.05).onTrue(Commands.runOnce(() -> {
            claw.setGamePiece(SuperstructureStates.ClawGamePiece.CUBE);
        }));

        coDriverController.back().onTrue(Commands.runOnce(() -> arm.setArmSide(
                (arm.getArmSide() == SuperstructureStates.ArmSide.ENERGY_CHAIN_TOP)
                        ? SuperstructureStates.ArmSide.ENERGY_CHAIN_BOTTOM
                        : SuperstructureStates.ArmSide.ENERGY_CHAIN_TOP
        )).andThen(() -> {
            arm.setDesiredState(SuperstructureStates.ArmState.STANDBY);
            claw.setDesiredState(SuperstructureStates.ClawState.STANDBY);
        }));

        coDriverController.start().onTrue(Commands.runOnce(() -> {
            arm.setDesiredState(SuperstructureStates.ArmState.STANDBY);
            claw.setDesiredState(SuperstructureStates.ClawState.STANDBY);
        }));

        // ArmClawTeleop
        ArmClawTeleop.addMapping(
                coDriverController.a(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.SCORING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .withClawState(SuperstructureStates.ClawState.INTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.x(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.SCORING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.SINGLE_SUB_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.SINGLE_SUB_CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .withClawState(SuperstructureStates.ClawState.INTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.y(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.SCORING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.DOUBLE_SUB_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.DOUBLE_SUB_CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .withClawState(SuperstructureStates.ClawState.INTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.b(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.SCORING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.TIPPED_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .withClawState(SuperstructureStates.ClawState.INTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.povUp(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.INTAKING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_HIGH_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_HIGH_CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .waitUntilState(
                                List.of(
                                        SuperstructureStates.ArmState.EXTENDED_HIGH_CONE,
                                        SuperstructureStates.ArmState.EXTENDED_HIGH_CUBE
                                )
                        )
                        .withClawState(SuperstructureStates.ClawState.OUTTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.povRight(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.INTAKING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_MID_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_MID_CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .waitUntilState(
                                List.of(
                                        SuperstructureStates.ArmState.EXTENDED_MID_CONE,
                                        SuperstructureStates.ArmState.EXTENDED_MID_CUBE
                                )
                        )
                        .withClawState(SuperstructureStates.ClawState.OUTTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.povLeft(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.INTAKING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_MID_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_MID_CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .waitUntilState(
                                List.of(
                                        SuperstructureStates.ArmState.EXTENDED_MID_CONE,
                                        SuperstructureStates.ArmState.EXTENDED_MID_CUBE
                                )
                        )
                        .withClawState(SuperstructureStates.ClawState.OUTTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.povDown(),
                new ArmClawCommand.Builder(arm, claw)
                        .endIfInState(SuperstructureStates.ClawArmStateType.INTAKING)
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_LOW_CONE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CONE
                        )
                        .withConditionalGamePieceArmState(
                                SuperstructureStates.ArmState.EXTENDED_LOW_CUBE,
                                claw.getGamePiece(),
                                SuperstructureStates.ClawGamePiece.CUBE
                        )
                        .waitUntilState(
                                List.of(
                                        SuperstructureStates.ArmState.EXTENDED_LOW_CONE,
                                        SuperstructureStates.ArmState.EXTENDED_LOW_CUBE
                                )
                        )
                        .withClawState(SuperstructureStates.ClawState.OUTTAKE)
                        .build()
        );

        ArmClawTeleop.addMapping(
                coDriverController.start(),
                new ArmClawCommand.Builder(arm, claw)
                        .withArmClawStates(
                                SuperstructureStates.ArmState.STANDBY,
                                SuperstructureStates.ClawState.STANDBY
                        )
                        .build()
        );
    }

    public static void clear() {
        ArmClawTeleop.removeAllMappings();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
