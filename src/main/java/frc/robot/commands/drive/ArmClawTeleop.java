package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.teleop.ArmClawCommand;

import java.util.HashMap;
import java.util.Map;

public class ArmClawTeleop extends Command {
    private static final Map<Trigger, ArmClawCommand> controllerMappings = new HashMap<>();

    private final Arm arm;
    private final Claw claw;

    public ArmClawTeleop(
            final Arm arm,
            final Claw claw
    ) {
        this.arm = arm;
        this.claw = claw;

        addRequirements(arm, claw);
    }

    public static void addMapping(
            final Trigger buttonPressedTrigger,
            final ArmClawCommand armClawCommand
    ) {
        if (controllerMappings.containsKey(buttonPressedTrigger)) {
            throw new RuntimeException("Attempted to map the same ButtonPressedSupplier twice!");
        }

        controllerMappings.put(buttonPressedTrigger, armClawCommand);
    }

    public static void removeMapping(final Trigger buttonPressedTrigger) {
        controllerMappings.remove(buttonPressedTrigger);
    }

    public static void removeAllMappings() {
        controllerMappings.clear();
    }

    @Override
    public void initialize() {
        final SuperstructureStates.ClawState clawState = claw.getDesiredState();
        if (clawState != SuperstructureStates.ClawState.STANDBY) {
            claw.setDesiredState(SuperstructureStates.ClawState.STANDBY);
        }

        for (final Map.Entry<Trigger, ArmClawCommand> mapping : controllerMappings.entrySet()) {
            final Trigger mappedTrigger = mapping.getKey();
            final ArmClawCommand mappedCommand = mapping.getValue();

            mappedTrigger.onTrue(Commands.runOnce(() -> {
                if (!CommandScheduler.getInstance().isScheduled(mappedCommand)) {
                    mappedCommand.schedule();
                }
            }));
        }
    }
}
