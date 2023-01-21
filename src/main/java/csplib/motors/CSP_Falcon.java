package csplib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class CSP_Falcon extends WPI_TalonFX implements CSP_Motor {

    private final double CPR = 2048.0;

    private double posScalar = 1.0;
    private double velScalar = 1.0;

    public CSP_Falcon(int id, String canBus) {
        super(id, canBus);
    }

    public CSP_Falcon(int id) {
        super(id, "rio");
    }

    public void init() {
        super.configFactoryDefault();
        super.clearStickyFaults();
        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        setEncoder(0.0);
    }

    public void set(double percent) {
        super.set(percent);
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public void setVelocity(double velocity) {
        super.set(ControlMode.Velocity, velocity);
    }

    public void setPosition(double position) {
        super.set(ControlMode.Position, position);
    }

    public void setInverted(boolean inverted) {
        super.setInverted(inverted);
    }

    public void setBrake(boolean brake) {
        super.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setRampRate(double rampRate) {
        super.configClosedloopRamp(rampRate);
        super.configOpenloopRamp(rampRate);
    }   

    public void setPIDF(double kP, double kI, double kD, double kF) {
        super.config_kP(0, kP);
        super.config_kI(0, kI);
        super.config_kD(0, kD);
        super.config_kF(0, kF);
    }

    public void setEncoder(double position) {
        super.setSelectedSensorPosition(position);
    }

    public void setPositionScalar(double scalar) {
        this.posScalar = scalar;
    }

    public void setVelocityScalar(double scalar) {
        this.velScalar = scalar;
    }
 
    public double getPosition() {
        return (super.getSelectedSensorPosition() / CPR) * posScalar;
    }

    public double getVelocity() {
        return ((super.getSelectedSensorVelocity() / CPR * 10) * 60) * velScalar;
    }

    public double getCurrent() {
        return super.getStatorCurrent();
    }

    public double getTemperature() {
        return super.getTemperature();
    }

    public double getCPR() {
        return CPR;
    }
}
