package frc.robot.subsystems.claw;

import frc.robot.utils.SuperstructureStates;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        public double wristEncoderPositionRots = 0.0;
        public double wristEncoderVelocityRotsPerSec = 0.0;
        public double wristMotorDutyCycle = 0.0;
        public double wristMotorCurrentAmps = 0.0;
        public double wristMotorTempCelsius = 0.0;

        public double rollerEncoderVelocityRotsPerSec = 0.0;
        public double rollerMotorDutyCycle = 0.0;
        public double rollerMotorCurrentAmps = 0.0;
        public double rollerMotorTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs Logged class of IOInputs
     * @see ClawIO.ClawIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ClawIO.ClawIOInputs inputs) {
    }

    /**
     * Periodic call to update the elevator,
     * this could include but isn't limited to updating states and motor setpoints
     */
    default void periodic() {
    }

    /**
     * Config call, should only be called once
     */
    default void config() {
    }

    /**
     * Called <b>after</b> {@link ClawIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {
    }

    /**
     * Sets the desired state of the elevator to a supplied {@link SuperstructureStates.ClawState}
     *
     * @param desiredState the new {@link SuperstructureStates.ClawState}
     * @param gamePiece    the desired/current game piece {@link SuperstructureStates.ClawGamePiece}
     * @see SuperstructureStates.ClawState
     */
    default void setDesiredState(
            final SuperstructureStates.ClawState desiredState,
            final SuperstructureStates.ClawGamePiece gamePiece
    ) {
    }
}
