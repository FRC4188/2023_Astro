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
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.sim.LimitSwitchSim;
import frc.robot.wrappers.motors.TitanSparkMAX;

//TODO This doesnt work cause the math for each stage hasnt been done.
public class ArmIOSim implements ArmIO {
    //TODO There is a lot of math in that class and tbh im too lazy but i also dont have your cad so thats gonna be my excuse
//    private final ArmSimSolver armSimSolver;
    private final DeltaTime deltaTime;

    private final TitanSparkMAX shoulderMotorLeader;
    private final TitanSparkMAX shoulderMotorFollower;
    private final CANcoder shoulderEncoder;

    private final TalonFX telescopeMotor;
    private final LimitSwitchSim telescopeLimitSwitchSim;

    //TODO Wrist really should be in claw since it is part of the claw but since you have flippy thing im too lazy to make a getter or some other jank way of flipping.
    // Ur gonna wanna prob use a doublejointedarmsim for this but that sounds like a lot of work and math so :/
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
    private final StatusSignal<Double> _telescopeTorqueCurrent;
    private final StatusSignal<Double> _telescopeTemp;

    public ArmIOSim(final HardwareConstants.ArmConstants constants) {
        this.deltaTime = new DeltaTime();

        this.shoulderMotorLeader = new TitanSparkMAX(
                constants.shoulderLeaderMotorId(), CANSparkLowLevel.MotorType.kBrushless
        );
        this.shoulderMotorFollower = new TitanSparkMAX(
                constants.shoulderFollowerMotorId(), CANSparkLowLevel.MotorType.kBrushless
        );
        this.shoulderEncoder = new CANcoder(constants.shoulderEncoderId());

        this.telescopeMotor = new TalonFX(constants.telescopeMotorId());
        this.telescopeLimitSwitchSim = new LimitSwitchSim(new DigitalInput(constants.telescopeLimitSwitchDIOChannel()));
        this.telescopeLimitSwitchSim.setInitialized(false);

        this.wristMotor = new TitanSparkMAX(
                constants.wristMotorId(), CANSparkLowLevel.MotorType.kBrushless
        );

//        this.armSimSolver = new ArmSimSolver(
//                shoulderMotorLeader,
//                shoulderMotorFollower,
//                shoulderEncoder,
//                telescopeMotor,
//        );

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
        this._telescopeTorqueCurrent = telescopeMotor.getTorqueCurrent();
        this._telescopeTemp = telescopeMotor.getDeviceTemp();
    }


    private void updateSimulation() {
//        armSimSolver.update(deltaTime.get());
        telescopeLimitSwitchSim.setValue(telescopePositionRots == 0); //TODO idk where your limitswitch is or when it is supposed to toggle.
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _shoulderPosition,
                _shoulderVelocity,
                _telescopePosition,
                _telescopeVelocity,
                _telescopeDutyCycle,
                _telescopeTorqueCurrent,
                _telescopeTemp
        );

        updateSimulation();

        shoulderMotorLeader.set(
                shoulderPID.calculate(
                        _shoulderPosition.getValue(),
                        shoulderPositionRots
                )
        );
        if (telescopeLimitSwitchSim.get()) { //TODO really bruh??!?
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
//        final Arm.ArmPoseState simState = armSimSolver.getArmPoseState();
//        simState.log(Arm.logKey + "SimState");

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
        inputs.telescopeMotorCurrentAmps = _telescopeTorqueCurrent.getValue();
        inputs.telescopeMotorTempCelsius = _telescopeTemp.getValue();

        final RelativeEncoder wristEncoder = wristMotor.getEncoder();
        inputs.wristEncoderPositionRots = wristEncoder.getPosition();
        inputs.wristEncoderVelocityRotsPerSec = wristEncoder.getVelocity();
        inputs.wristMotorDutyCycle = wristMotor.getAppliedOutput();
        inputs.wristMotorCurrentAmps = wristMotor.getOutputCurrent();
        inputs.wristMotorTempCelsius = wristMotor.getMotorTemperature();

        inputs.telescopeLimitSwitch = telescopeLimitSwitchSim.get();
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
        wristMotor.getEncoder().setPosition(126.4748);
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
