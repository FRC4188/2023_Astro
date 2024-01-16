package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.PIDUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ArmIOReal implements ArmIO {
    private final TitanSparkMAX shoulderMotorLeader;
    private final TitanSparkMAX shoulderMotorFollower;
    private final CANcoder shoulderEncoder;

    private final TalonFX telescopeMotor;
    private final DigitalInput telescopeLimitSwitch;

    private final TitanSparkMAX wristMotor;

    private final ProfiledPIDController shoulderPID;
    private final MotionMagicVoltage telescopeMotionMagicVoltage;
    private final ProfiledPIDController wristPID;

    private SuperstructureStates.ArmState desiredState = SuperstructureStates.ArmState.STANDBY;
    private SuperstructureStates.ArmState lastDesiredState = desiredState;
    private double shoulderPositionRots = desiredState.getArmPositions().shoulderNormalRotations();
    private double telescopePositionRots = desiredState.getArmPositions().telescopeNormalRotations();
    private double wristPositionRots = desiredState.getArmPositions().wristNormalRotations();

    private final StatusSignal<Double> _shoulderPosition;
    private final StatusSignal<Double> _shoulderVelocity;
    private final StatusSignal<Double> _telescopePosition;
    private final StatusSignal<Double> _telescopeVelocity;
    private final StatusSignal<Double> _telescopeDutyCycle;
    private final StatusSignal<Double> _telescopeStatorCurrent;
    private final StatusSignal<Double> _telescopeTemp;

    public ArmIOReal(final HardwareConstants.ArmConstants constants) {
        this.shoulderMotorLeader = new TitanSparkMAX(
                constants.shoulderLeaderMotorId(), CANSparkLowLevel.MotorType.kBrushless
        );
        this.shoulderMotorFollower = new TitanSparkMAX(
                constants.shoulderFollowerMotorId(), CANSparkLowLevel.MotorType.kBrushless
        );
        this.shoulderEncoder = new CANcoder(constants.shoulderEncoderId(), constants.armCANBus());

        this.telescopeMotor = new TalonFX(constants.telescopeMotorId(), constants.armCANBus());
        this.telescopeLimitSwitch = new DigitalInput(constants.telescopeLimitSwitchDIOChannel());

        this.wristMotor = new TitanSparkMAX(constants.wristMotorId(), CANSparkLowLevel.MotorType.kBrushless);

        this.shoulderPID = new ProfiledPIDController(
                0.02, 0, 0,
                new TrapezoidProfile.Constraints(
                        1000, 720
                )
        );
        this.telescopeMotionMagicVoltage = new MotionMagicVoltage(0);

        this.wristPID = new ProfiledPIDController(
                0.03, 0, 0,
                new TrapezoidProfile.Constraints(
                        1000, 520
                )
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                shoulderPID,
                shoulderEncoder.getAbsolutePosition(),
                shoulderEncoder.getVelocity()
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                wristPID,
                wristMotor.getEncoder().getPosition(),
                wristMotor.getEncoder().getVelocity()
        );

        this._shoulderPosition = shoulderEncoder.getPosition();
        this._shoulderVelocity = shoulderEncoder.getVelocity();
        this._telescopePosition = telescopeMotor.getPosition();
        this._telescopeVelocity = telescopeMotor.getVelocity();
        this._telescopeDutyCycle = telescopeMotor.getDutyCycle();
        this._telescopeStatorCurrent = telescopeMotor.getStatorCurrent();
        this._telescopeTemp = telescopeMotor.getDeviceTemp();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _shoulderPosition,
                _shoulderVelocity,
                _telescopePosition,
                _telescopeVelocity,
                _telescopeDutyCycle,
                _telescopeStatorCurrent,
                _telescopeTemp
        );

        shoulderMotorLeader.set(
                shoulderPID.calculate(
                        _shoulderPosition.getValue(),
                        shoulderPositionRots
                )
        );
        if (telescopeLimitSwitch.get()) { //TODO really bruh??!?
            telescopeMotor.stopMotor();
        } else {
            telescopeMotor.setControl(telescopeMotionMagicVoltage.withPosition(telescopePositionRots));
        }

        wristMotor.set(
                wristPID.calculate(
                        wristMotor.getEncoder().getPosition(),
                        wristPositionRots
                )
        );
    }

    @Override
    public void updateInputs(final ArmIOInputs inputs) {
        inputs.shoulderEncoderPositionRots = _shoulderPosition.getValue();
        inputs.shoulderEncoderVelocityRotsPerSec = _shoulderVelocity.getValue();
        inputs.shoulderMotorDutyCycle = shoulderMotorLeader.getAppliedOutput();
        inputs.shoulderMotorCurrentsAmps = new double[]{
                shoulderMotorLeader.getOutputCurrent(), shoulderMotorFollower.getOutputCurrent()
        };
        inputs.shoulderMotorTempsCelsius = new double[]{
                shoulderMotorLeader.getMotorTemperature(), shoulderMotorFollower.getMotorTemperature()
        };

        inputs.telescopeEncoderPositionRots = _telescopePosition.getValue();
        inputs.telescopeEncoderVelocityRotsPerSec = _telescopeVelocity.getValue();
        inputs.telescopeMotorDutyCycle = _telescopeDutyCycle.getValue();
        inputs.telescopeMotorCurrentAmps = _telescopeStatorCurrent.getValue();
        inputs.telescopeMotorTempCelsius = _telescopeTemp.getValue();

        final RelativeEncoder wristEncoder = wristMotor.getEncoder();
        inputs.wristEncoderPositionRots = wristEncoder.getPosition();
        inputs.wristEncoderVelocityRotsPerSec = wristEncoder.getVelocity();
        inputs.wristMotorDutyCycle = wristMotor.getAppliedOutput();
        inputs.wristMotorCurrentAmps = wristMotor.getOutputCurrent();
        inputs.wristMotorTempCelsius = wristMotor.getMotorTemperature();

        inputs.telescopeLimitSwitch = telescopeLimitSwitch.get();
    }

    @Override
    public void config() {
        shoulderMotorLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shoulderMotorFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);

        //TODO TUNE ALL CURRENT LIMITS
        shoulderMotorLeader.setSmartCurrentLimit(30);
        shoulderMotorFollower.setSmartCurrentLimit(30);

        shoulderMotorLeader.setInverted(true);
        shoulderMotorFollower.follow(shoulderMotorLeader, false);

        final CANcoderConfiguration shoulderEncoderConfig = new CANcoderConfiguration();
        shoulderEncoderConfig.MagnetSensor.MagnetOffset = Units.degreesToRotations(-66.884765625);
        shoulderEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        shoulderEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        shoulderEncoder.getConfigurator().apply(shoulderEncoderConfig);

        final TalonFXConfiguration telescopeMotorConfig = new TalonFXConfiguration();
        telescopeMotorConfig.Slot0 = new Slot0Configs()
                .withKP(68)
                .withKS(0.4495)
                .withKG(1.21)
                .withKV(3.8186);
        telescopeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        telescopeMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
        telescopeMotorConfig.MotionMagic.MotionMagicAcceleration = 35;
        telescopeMotorConfig.MotionMagic.MotionMagicJerk = 35;
        telescopeMotorConfig.CurrentLimits.StatorCurrentLimit = 35;
        telescopeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
//        telescopeMotorConfig.Feedback.SensorToMechanismRatio = //TODO idk what your gear ratio is if you have one
        telescopeMotor.getConfigurator().apply(telescopeMotorConfig);

        //Neo 550?
        wristMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(15);
        wristMotor.getEncoder().setPosition(126.4748); //That's oddly specific
    }

    @Override
    public void setDesiredState(
            final SuperstructureStates.ArmState desiredState,
            final SuperstructureStates.ArmSide armSide
    ) {
        this.lastDesiredState = this.desiredState;
        this.desiredState = desiredState;

        //TODO CHECK
        if (armSide == SuperstructureStates.ArmSide.ENERGY_CHAIN_BOTTOM) {
            this.shoulderPositionRots = desiredState.getArmPositions().shoulderFlippedRotations();
            this.telescopePositionRots = desiredState.getArmPositions().telescopeFlippedRotations();
            this.wristPositionRots = desiredState.getArmPositions().wristFlippedRotations();
        } else {
            this.shoulderPositionRots = desiredState.getArmPositions().shoulderNormalRotations();
            this.telescopePositionRots = desiredState.getArmPositions().telescopeNormalRotations();
            this.wristPositionRots = desiredState.getArmPositions().wristNormalRotations();
        }
    }
}
