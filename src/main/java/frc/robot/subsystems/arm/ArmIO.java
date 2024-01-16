package frc.robot.subsystems.arm;

import frc.robot.utils.SuperstructureStates;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double shoulderEncoderPositionRots = 0.0;
        public double shoulderEncoderVelocityRotsPerSec = 0.0;
        public double shoulderMotorDutyCycle = 0.0;
        public double[] shoulderMotorCurrentsAmps = new double[]{0.0, 0.0};
        public double[] shoulderMotorTempsCelsius = new double[]{0.0, 0.0};

        public double telescopeEncoderPositionRots = 0.0;
        public double telescopeEncoderVelocityRotsPerSec = 0.0;
        public double telescopeMotorDutyCycle = 0.0;
        public double telescopeMotorCurrentAmps = 0.0;
        public double telescopeMotorTempCelsius = 0.0;

        public double wristEncoderPositionRots = 0.0;
        public double wristEncoderVelocityRotsPerSec = 0.0;
        public double wristMotorDutyCycle = 0.0;
        public double wristMotorCurrentAmps = 0.0;
        public double wristMotorTempCelsius = 0.0;

        public boolean telescopeLimitSwitch = false;
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs Logged class of IOInputs
     * @see ArmIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ArmIOInputs inputs) {
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
     * Called <b>after</b> {@link ArmIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {
    }

    /**
     * Sets the desired state of the elevator to a supplied {@link SuperstructureStates.ArmState}
     *
     * @param desiredState the new {@link SuperstructureStates.ArmState}
     * @see SuperstructureStates.ArmState
     */
    default void setDesiredState(
            final SuperstructureStates.ArmState desiredState,
            final SuperstructureStates.ArmSide armSide
    ) {}
}
