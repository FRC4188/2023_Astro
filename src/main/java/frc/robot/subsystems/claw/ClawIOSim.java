package frc.robot.subsystems.claw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.SuperstructureStates;

public class ClawIOSim implements ClawIO {
    private final TalonFX rollerMotor;

    private SuperstructureStates.ClawState desiredState = SuperstructureStates.ClawState.STANDBY;
    private SuperstructureStates.ClawState lastDesiredState = desiredState;
    private double rollerDutyCycle = desiredState.rollerDutyCycle();

    private final StatusSignal<Double> _rollerVelocity;
    private final StatusSignal<Double> _rollerDutyCycle;
    private final StatusSignal<Double> _rollerStatorCurrent;
    private final StatusSignal<Double> _rollerTemp;

    public ClawIOSim(final HardwareConstants.ClawConstants clawConstants) {
        this.rollerMotor = new TalonFX(clawConstants.rollerMotorId(), clawConstants.clawCANBus());

        this._rollerVelocity = rollerMotor.getVelocity();
        this._rollerDutyCycle = rollerMotor.getDutyCycle();
        this._rollerStatorCurrent = rollerMotor.getStatorCurrent();
        this._rollerTemp = rollerMotor.getDeviceTemp();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _rollerVelocity,
                _rollerDutyCycle,
                _rollerStatorCurrent,
                _rollerTemp
        );

        rollerMotor.set(rollerDutyCycle);
    }

    @Override
    public void updateInputs(final ClawIO.ClawIOInputs inputs) {
        inputs.rollerEncoderVelocityRotsPerSec = _rollerVelocity.getValue();
        inputs.rollerMotorDutyCycle = _rollerDutyCycle.getValue();
        inputs.rollerMotorCurrentAmps = _rollerStatorCurrent.getValue();
        inputs.rollerMotorTempCelsius = _rollerTemp.getValue();
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.StatorCurrentLimit = 30;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    @Override
    public void setDesiredState(
            final SuperstructureStates.ClawState desiredState,
            final SuperstructureStates.ClawGamePiece gamePiece
    ) {
        this.lastDesiredState = this.desiredState;
        this.desiredState = desiredState;

        this.rollerDutyCycle = desiredState.rollerDutyCycle() * gamePiece.getScalar();
    }
}
