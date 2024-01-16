package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    protected static final String logKey = "Arm";

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private SuperstructureStates.ArmState desiredState = SuperstructureStates.ArmState.STANDBY;
    private SuperstructureStates.ArmState currentState = desiredState;
    private boolean transitioning = false;
    private SuperstructureStates.ArmSide armSide = SuperstructureStates.ArmSide.ENERGY_CHAIN_TOP;

    public Arm(final Constants.RobotMode mode, final HardwareConstants.ArmConstants armConstants) {
        this.armIO = switch (mode) {
            case REAL -> new ArmIOReal(armConstants);
            case SIM -> new ArmIOSim(armConstants);
            case REPLAY -> new ArmIO() {
            };
        };

        this.inputs = new ArmIOInputsAutoLogged();

        this.armIO.config();
        this.armIO.initialize();

        setDesiredState(desiredState);
    }

    @Override
    public void periodic() {
        final double armIOPeriodicStart = Logger.getRealTimestamp();
        armIO.periodic();

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - armIOPeriodicStart)
        );

        armIO.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.recordOutput(logKey + "/AtDesiredState", isAtDesiredState());
        Logger.recordOutput(logKey + "/IsTransitioning", transitioning);
        Logger.recordOutput(logKey + "/ArmSide", armSide.toString());
    }

    /**
     * Sets the desired {@link frc.robot.utils.SuperstructureStates.ArmState}.
     *
     * @param desiredState the new desired {@link frc.robot.utils.SuperstructureStates.ArmState}
     * @implNote This will put the system into a transitioning state if the new desiredState is != to the currentState
     * @see ArmIO#setDesiredState(SuperstructureStates.ArmState, frc.robot.utils.SuperstructureStates.ArmSide)
     */
    public void setDesiredState(final SuperstructureStates.ArmState desiredState) {
        this.desiredState = desiredState;
        if (desiredState != currentState) {
            this.transitioning = true;
        }

        armIO.setDesiredState(desiredState, armSide);
    }

    public void setArmSide(final SuperstructureStates.ArmSide armSide) {
        this.armSide = armSide;
    }

    public SuperstructureStates.ArmSide getArmSide() {
        return armSide;
    }

    public SuperstructureStates.ArmState getDesiredState() {
        return desiredState;
    }

    public SuperstructureStates.ArmState getCurrentState() {
        return currentState;
    }

    public SuperstructureStates.ArmState getCurrentStateWithNullAsTransition() {
        return transitioning ? null : currentState;
    }

    /**
     * Get if the system is at its desired {@link frc.robot.utils.SuperstructureStates.ArmState}.
     *
     * <p>This <b>should</b> be called periodically to update the currentState of the system
     * which will ensure that anything reading from currentState directly without interacting with this method
     * will receive the correct currentState, however, this isn't required if the only interaction with the
     * currentState is through this method (which will update the currentState before returning a result)</p>
     *
     * @return true if the system is at the desired {@link frc.robot.utils.SuperstructureStates.ArmState},
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
        //TODO CHECK ENERGY CHAIN SIDE
        return MathUtils.withinTolerance(
                inputs.shoulderEncoderPositionRots,
                armSide == SuperstructureStates.ArmSide.ENERGY_CHAIN_TOP ?
                        desiredState.getArmPositions().shoulderNormalRotations() :
                        desiredState.getArmPositions().shoulderFlippedRotations(),
                0.4
        ) && MathUtils.withinTolerance(
                inputs.telescopeEncoderPositionRots,
                armSide == SuperstructureStates.ArmSide.ENERGY_CHAIN_TOP ?
                        desiredState.getArmPositions().telescopeNormalRotations() :
                        desiredState.getArmPositions().telescopeFlippedRotations(),
                0.4
        ) && MathUtils.withinTolerance(
                inputs.wristEncoderPositionRots,
                armSide == SuperstructureStates.ArmSide.ENERGY_CHAIN_TOP ?
                        desiredState.getArmPositions().wristNormalRotations() :
                        desiredState.getArmPositions().wristFlippedRotations(),
                0.4
        );
    }

    public boolean isAtState(final SuperstructureStates.ArmState armState) {
        return !transitioning && currentState == armState;
    }
}
