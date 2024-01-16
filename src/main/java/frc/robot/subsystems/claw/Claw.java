package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    protected static final String logKey = "Claw";

    private final ClawIO clawIO;
    private final ClawIOInputsAutoLogged inputs;

    private SuperstructureStates.ClawState desiredState = SuperstructureStates.ClawState.STANDBY;
    private SuperstructureStates.ClawState currentState = desiredState;
    private SuperstructureStates.ClawGamePiece gamePiece = SuperstructureStates.ClawGamePiece.CONE;
    private boolean transitioning = false;

    public Claw(final Constants.RobotMode mode, final HardwareConstants.ClawConstants clawConstants) {
        this.clawIO = switch (mode) {
            case REAL -> new ClawIOReal(clawConstants);
            case SIM -> new ClawIOSim(clawConstants);
            case REPLAY -> new ClawIO() {
            };
        };

        this.inputs = new ClawIOInputsAutoLogged();

        this.clawIO.config();
        this.clawIO.initialize();

        setDesiredState(desiredState);
    }

    @Override
    public void periodic() {
        final double clawIOPeriodicStart = Logger.getRealTimestamp();
        clawIO.periodic();

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - clawIOPeriodicStart)
        );

        clawIO.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.recordOutput(logKey + "/AtDesiredState", isAtDesiredState());
        Logger.recordOutput(logKey + "/IsTransitioning", transitioning);
    }

    /**
     * Sets the desired {@link frc.robot.utils.SuperstructureStates.ClawState}.
     *
     * @param desiredState the new desired {@link frc.robot.utils.SuperstructureStates.ClawState}
     * @implNote This will put the system into a transitioning state if the new desiredState is != to the currentState
     * @see ClawIO#setDesiredState(SuperstructureStates.ClawState, SuperstructureStates.ClawGamePiece)
     */
    public void setDesiredState(final SuperstructureStates.ClawState desiredState) {
        this.desiredState = desiredState;
        if (desiredState != currentState) {
            this.transitioning = true;
        }

        clawIO.setDesiredState(desiredState, gamePiece);
    }

    public void setGamePiece(final SuperstructureStates.ClawGamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public SuperstructureStates.ClawGamePiece getGamePiece() {
        return gamePiece;
    }

    public SuperstructureStates.ClawState getDesiredState() {
        return desiredState;
    }

    public SuperstructureStates.ClawState getCurrentState() {
        return currentState;
    }

    public SuperstructureStates.ClawState getCurrentStateWithNullAsTransition() {
        return transitioning ? null : currentState;
    }

    /**
     * Get if the system is at its desired {@link frc.robot.utils.SuperstructureStates.ClawState}.
     *
     * <p>This <b>should</b> be called periodically to update the currentState of the system
     * which will ensure that anything reading from currentState directly without interacting with this method
     * will receive the correct currentState, however, this isn't required if the only interaction with the
     * currentState is through this method (which will update the currentState before returning a result)</p>
     *
     * @return true if the system is at the desired {@link frc.robot.utils.SuperstructureStates.ClawState},
     * false if not
     */
    public boolean isAtDesiredState() {
        if (currentState == desiredState && !transitioning) {
            return true;
        } else {
            final boolean isAtDesired = isAtDesiredStateInternal();
            if (isAtDesired) {
                this.currentState = desiredState;
                this.transitioning = false;
            }

            return isAtDesired;
        }
    }

    private boolean isAtDesiredStateInternal() {
        return MathUtils.withinTolerance(
                inputs.rollerMotorDutyCycle,
                desiredState.rollerDutyCycle(),
                0.2
        );
    }

    public boolean isAtState(final SuperstructureStates.ClawState clawState) {
        return !transitioning && currentState == clawState;
    }

    public boolean isGamePieceInIntake() {
        return inputs.rollerMotorCurrentAmps >= 25;
    }
}
