package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.utils.SuperstructureStates;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class ArmClawCommand extends SequentialCommandGroup {
    public ArmClawCommand(final Command... commands) {
        super(commands);
    }

    public static class CancelSequentialCommand extends Command {
        private final BooleanSupplier condition;
        private SequentialCommandGroup toCancelCommand;

        public CancelSequentialCommand(final BooleanSupplier condition) {
            this.condition = condition;
        }

        @SuppressWarnings("unused")
        public CancelSequentialCommand() {
            this(() -> true);
        }

        @SuppressWarnings("unused")
        public CancelSequentialCommand(final BooleanSupplier condition, final SequentialCommandGroup toCancelCommand) {
            this.condition = condition;
            setToCancelCommand(toCancelCommand);
        }

        public void setToCancelCommand(final SequentialCommandGroup toCancelCommand) {
            this.toCancelCommand = toCancelCommand;
            addRequirements(toCancelCommand.getRequirements().toArray(Subsystem[]::new));
        }

        public void cancelIfPresentAndCondition() {
            if (toCancelCommand != null && condition.getAsBoolean()) {
                toCancelCommand.cancel();
            }
        }

        @Override
        public void initialize() {
            cancelIfPresentAndCondition();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    @SuppressWarnings({"unused", "UnusedReturnValue"})
    public static class Builder {
        public static final double WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS = 5;

        private final List<Command> commands;
        private final Arm arm;
        private final Claw claw;

        private final Function<SuperstructureStates.ArmState, Boolean> isArmState;
        private final Function<SuperstructureStates.ArmState, Boolean> isDesiredArmState;

        private final Function<SuperstructureStates.ClawState, Boolean> isClawState;
        private final Function<SuperstructureStates.ClawState, Boolean> isDesiredClawState;

        private final Function<SuperstructureStates.ClawArmStateType, Boolean> isArmClawStateType;
        private final Function<SuperstructureStates.ClawArmStateType, Boolean> isDesiredArmClawStateType;

        public Builder(final Arm arm, final Claw claw) {
            this.arm = arm;
            this.claw = claw;

            this.isArmState = arm::isAtState;
            this.isDesiredArmState = (state) -> arm.getDesiredState() == state;

            this.isClawState = claw::isAtState;
            this.isDesiredClawState = (state) -> claw.getDesiredState() == state;

            this.isArmClawStateType = (state) -> {
                final SuperstructureStates.ArmState armState = arm.getCurrentStateWithNullAsTransition();
                final SuperstructureStates.ClawState clawState = claw.getCurrentStateWithNullAsTransition();

                return (armState != null && armState.getClawArmStateType() == state)
                        || (clawState != null && clawState.getClawArmStateType() == state);
            };

            this.isDesiredArmClawStateType = (state) -> (
                    (arm.getDesiredState().getClawArmStateType() == state)
                            || (claw.getDesiredState().getClawArmStateType() == state)
            );

            this.commands = new ArrayList<>();
        }

        /**
         * Adds a {@link SuperstructureStates.ArmState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ArmState} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionArmState the {@link SuperstructureStates.ArmState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ArmState
         */
        public Builder endIfInState(final SuperstructureStates.ArmState endConditionArmState) {
            commands.add(new CancelSequentialCommand(() -> isArmState.apply(endConditionArmState)));
            return this;
        }

        /**
         * Adds a {@link SuperstructureStates.ClawState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ClawState} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionClawState the {@link SuperstructureStates.ClawState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder endIfInState(final SuperstructureStates.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds a {@link SuperstructureStates.ClawArmStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ClawArmStateType} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionArmClawStateType the {@link SuperstructureStates.ClawArmStateType} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawArmStateType
         */
        public Builder endIfInState(final SuperstructureStates.ClawArmStateType endConditionArmClawStateType) {
            commands.add(
                    new CancelSequentialCommand(() -> isArmClawStateType.apply(endConditionArmClawStateType))
            );

            return this;
        }

        /**
         * Adds all states except a single {@link SuperstructureStates.ArmState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ArmState} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionArmState the {@link SuperstructureStates.ArmState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ArmState
         */
        public Builder endIfNotInState(final SuperstructureStates.ArmState endConditionArmState) {
            commands.add(new CancelSequentialCommand(() -> !isArmState.apply(endConditionArmState)));
            return this;
        }

        /**
         * Adds all states except a single {@link SuperstructureStates.ClawState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ClawState} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionClawState the {@link SuperstructureStates.ClawState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder endIfNotInState(final SuperstructureStates.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> !isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds all states except a single {@link SuperstructureStates.ClawArmStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ClawArmStateType} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionArmClawStateType the {@link SuperstructureStates.ClawArmStateType} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawArmStateType
         */
        public Builder endIfNotInState(final SuperstructureStates.ClawArmStateType endConditionArmClawStateType) {
            commands.add(
                    new CancelSequentialCommand(() -> !isArmClawStateType.apply(endConditionArmClawStateType))
            );
            return this;
        }

        /**
         * Adds a wait/delay with a specified amount of time.
         *
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see WaitCommand
         */
        public Builder wait(final double waitSeconds) {
            commands.add(Commands.waitSeconds(waitSeconds));
            return this;
        }

        public Builder waitUntilState(final List<SuperstructureStates.ArmState> armStates) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS),
                            Commands.waitUntil(
                                    () -> {
                                        for (final SuperstructureStates.ArmState armState : armStates) {
                                            if (isArmState.apply(armState)) {
                                                return true;
                                            }
                                        }
                                        return false;
                                    }
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ArmState} is equal to the
         * supplied {@link SuperstructureStates.ArmState} or until a timeout
         *
         * @param armState the {@link SuperstructureStates.ArmState} to wait for
         * @param timeoutSec    the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ArmState armState, final double timeoutSec) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(
                                    () -> isArmState.apply(armState)
                                            || !isDesiredArmState.apply(armState)
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ArmState} is equal to the
         * supplied {@link SuperstructureStates.ArmState} or until a timeout (default amount of time)
         *
         * @param armState the {@link SuperstructureStates.ArmState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ArmState armState) {
            waitUntilState(armState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ClawState} is equal to the
         * supplied {@link SuperstructureStates.ClawState} or until a timeout
         *
         * @param clawState  the {@link SuperstructureStates.ClawState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ClawState clawState, final double timeoutSec) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(
                                    () -> isClawState.apply(clawState) || !isDesiredClawState.apply(clawState)
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current  {@link SuperstructureStates.ClawState} is equal to the
         * supplied {@link SuperstructureStates.ClawState} or until a timeout (default amount of time)
         *
         * @param clawState the {@link SuperstructureStates.ClawState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ClawState clawState) {
            waitUntilState(clawState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ClawArmStateType} (of the arm OR claw)
         * is equal to the supplied {@link SuperstructureStates.ClawArmStateType} or until a timeout
         *
         * @param armClawStateType the {@link SuperstructureStates.ClawArmStateType} to wait for
         * @param timeoutSec            the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(
                final SuperstructureStates.ClawArmStateType armClawStateType,
                final double timeoutSec
        ) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(
                                    () -> isArmClawStateType.apply(armClawStateType)
                                            || !isDesiredArmClawStateType.apply(armClawStateType)
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ClawArmStateType} (of the arm OR claw)
         * is equal to the supplied {@link SuperstructureStates.ClawArmStateType}
         * or until a timeout (default amount of time)
         *
         * @param clawArmStateType the {@link SuperstructureStates.ClawArmStateType} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ClawArmStateType clawArmStateType) {
            waitUntilState(clawArmStateType, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ArmState}
         * and the current {@link SuperstructureStates.ClawState} are both
         * equal to the supplied {@link SuperstructureStates.ArmState}
         * and {@link SuperstructureStates.ClawState}, respectively; or until a timeout
         *
         * @param armState the {@link SuperstructureStates.ArmState} to wait for
         * @param clawState     the {@link SuperstructureStates.ClawState} to wait for
         * @param timeoutSec    the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilStates(
                final SuperstructureStates.ArmState armState,
                final SuperstructureStates.ClawState clawState,
                final double timeoutSec
        ) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.parallel(
                                    Commands.waitUntil(
                                            () -> isArmState.apply(armState)
                                                    || !isDesiredArmState.apply(armState)
                                    ),
                                    Commands.waitUntil(
                                            () -> isClawState.apply(clawState)
                                                    || !isDesiredClawState.apply(clawState)
                                    )
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ArmState}
         * and the current {@link SuperstructureStates.ClawState} are both
         * equal to the supplied {@link SuperstructureStates.ArmState}
         * and {@link SuperstructureStates.ClawState}, respectively; or until a timeout (default amount of time)
         *
         * @param armState the {@link SuperstructureStates.ArmState} to wait for
         * @param clawState     the {@link SuperstructureStates.ClawState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilStates(final SuperstructureStates.ArmState armState, final SuperstructureStates.ClawState clawState) {
            waitUntilStates(armState, clawState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Adds a conditional wait/delay with a specified amount of time that only runs if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ArmState} matches the
         * supplied {@link SuperstructureStates.ArmState}.
         *
         * <p>A conditional command is specified by a command that is only ran if certain condition is met.</p>
         *
         * @param conditionalArmState the conditional {@link SuperstructureStates.ArmState}
         * @param waitSeconds              the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see SuperstructureStates.ArmState
         * @see WaitCommand
         */
        public Builder waitIfState(
                final SuperstructureStates.ArmState conditionalArmState,
                final double waitSeconds
        ) {
            commands.add(Commands.either(
                    Commands.waitSeconds(waitSeconds),
                    Commands.none(),
                    () -> isArmState.apply(conditionalArmState)
            ));
            return this;
        }

        /**
         * Adds a conditional wait/delay with a specified amount of time that only runs if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ClawState} matches the
         * supplied {@link SuperstructureStates.ClawState}.
         *
         * <p>A conditional command is specified by a command that is only ran if certain condition is met.</p>
         *
         * @param conditionalClawState the conditional {@link SuperstructureStates.ClawState}
         * @param waitSeconds          the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         * @see WaitCommand
         */
        public Builder waitIfState(
                final SuperstructureStates.ClawState conditionalClawState,
                final double waitSeconds
        ) {
            commands.add(Commands.either(
                    Commands.waitSeconds(waitSeconds),
                    Commands.none(),
                    () -> isClawState.apply(conditionalClawState)
            ));
            return this;
        }

        /**
         * Adds a desired {@link SuperstructureStates.ArmState}.
         *
         * @param armState the desired {@link SuperstructureStates.ArmState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ArmState
         */
        public Builder withArmState(
                final SuperstructureStates.ArmState armState
        ) {
            commands.add(Commands.runOnce(() -> arm.setDesiredState(armState)));
            return this;
        }

        /**
         * Adds a conditionally desired {@link SuperstructureStates.ArmState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ArmState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ArmState} matches
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalNotArmState(SuperstructureStates.ArmState, SuperstructureStates.ArmState, SuperstructureStates.ArmSide)}</p>
         *
         * @param conditionalArmState the conditional {@link SuperstructureStates.ArmState}
         * @param desiredArmState     the desired {@link SuperstructureStates.ArmState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ArmState
         */
        public Builder withConditionalArmState(
                final SuperstructureStates.ArmState conditionalArmState,
                final SuperstructureStates.ArmState desiredArmState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (isArmState.apply(conditionalArmState)) {
                    arm.setDesiredState(desiredArmState);
                }
            }));
            return this;
        }

        public Builder withConditionalGamePieceArmState(
                final SuperstructureStates.ArmState desiredArmState,
                final SuperstructureStates.ClawGamePiece currentGamePiece,
                final SuperstructureStates.ClawGamePiece desiredGamePiece
                ) {
            commands.add(Commands.runOnce(() -> {
                if (currentGamePiece == desiredGamePiece) {
                    arm.setDesiredState(desiredArmState);
                }
            }));
            return this;
        }

        /**
         * Adds a conditionally-inverted desired {@link SuperstructureStates.ArmState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ArmState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ArmState} does <b>NOT</b> match
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalArmState(SuperstructureStates.ArmState, SuperstructureStates.ArmState)}</p>
         *
         * @param conditionalArmState the conditional {@link SuperstructureStates.ArmState}
         * @param desiredArmState     the desired {@link SuperstructureStates.ArmState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ArmState
         */
        public Builder withConditionalNotArmState(
                final SuperstructureStates.ArmState conditionalArmState,
                final SuperstructureStates.ArmState desiredArmState,
                final SuperstructureStates.ArmSide armSide
        ) {
            commands.add(Commands.runOnce(() -> {
                if (!isArmState.apply(conditionalArmState)) {
                    arm.setDesiredState(desiredArmState);
                }
            }));
            return this;
        }

        /**
         * Adds a desired {@link SuperstructureStates.ClawState}.
         *
         * @param clawState the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder withClawState(
                final SuperstructureStates.ClawState clawState
        ) {
            commands.add(Commands.runOnce(() -> claw.setDesiredState(clawState)));
            return this;
        }

        /**
         * Adds a conditionally desired {@link SuperstructureStates.ClawState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ClawState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ClawState} matches
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalNotClawState(SuperstructureStates.ClawState, SuperstructureStates.ClawState)}</p>
         *
         * @param conditionalClawState the conditional {@link SuperstructureStates.ClawState}
         * @param desiredClawState     the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder withConditionalClawState(
                final SuperstructureStates.ClawState conditionalClawState,
                final SuperstructureStates.ClawState desiredClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (isClawState.apply(conditionalClawState)) {
                    claw.setDesiredState(desiredClawState);
                }
            }));
            return this;
        }

        public Builder withConditionalGamePieceClawState(
                final SuperstructureStates.ClawState desiredClawState,
                final SuperstructureStates.ClawGamePiece currentGamePiece,
                final SuperstructureStates.ClawGamePiece desiredGamePiece
        ) {
            commands.add(Commands.runOnce(() -> {
                if (currentGamePiece == desiredGamePiece) {
                    claw.setDesiredState(desiredClawState);
                }
            }));
            return this;
        }

        /**
         * Adds a conditionally-inverted desired {@link SuperstructureStates.ClawState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ClawState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ClawState} does <b>NOT</b> match
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalClawState(SuperstructureStates.ClawState, SuperstructureStates.ClawState)}</p>
         *
         * @param conditionalClawState the conditional {@link SuperstructureStates.ClawState}
         * @param desiredClawState     the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder withConditionalNotClawState(
                final SuperstructureStates.ClawState conditionalClawState,
                final SuperstructureStates.ClawState desiredClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (!isClawState.apply(conditionalClawState)) {
                    claw.setDesiredState(desiredClawState);
                }
            }));
            return this;
        }

        /**
         * Adds a desired {@link SuperstructureStates.ArmState} and {@link SuperstructureStates.ClawState}.
         *
         * @param armState the desired {@link SuperstructureStates.ArmState}
         * @param clawState     the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see Builder#withArmState(SuperstructureStates.ArmState)
         * @see Builder#withClawState(SuperstructureStates.ClawState)
         * @see SuperstructureStates.ArmState
         * @see SuperstructureStates.ClawState
         */
        public Builder withArmClawStates(
                final SuperstructureStates.ArmState armState,
                final SuperstructureStates.ClawState clawState
        ) {
            commands.add(Commands.runOnce(() -> {
                arm.setDesiredState(armState);
                claw.setDesiredState(clawState);
            }));

            return this;
        }


        /**
         * Builds this {@link Builder}, producing a {@link ArmClawCommand}
         *
         * @return the produced {@link ArmClawCommand}
         */
        public ArmClawCommand build() {
            final ArmClawCommand armClawCommand = new ArmClawCommand();

            for (final Command command : commands) {
                if (command instanceof CancelSequentialCommand) {
                    ((CancelSequentialCommand) command).setToCancelCommand(armClawCommand);
                }
            }

            armClawCommand.addCommands(commands.toArray(Command[]::new));
            return armClawCommand;
        }
    }
}
