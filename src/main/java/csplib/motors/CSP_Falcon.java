package csplib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

<<<<<<< HEAD
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

=======
public class CSP_Falcon extends WPI_TalonFX implements CSP_Motor {

    /**
     * Creates a CSP_Falcon object
     * @param id CAN ID of the Falcon 500
     * @param canBus name of the CAN Bus the Falcon is on
     */
    public CSP_Falcon(int id, String canBus) {
        super(id, canBus);
        init();
    }

    /**
     * Creates a CSP_Falcon object with assumed Roborio CAN Bus 
     * @param id CAN ID of the Falcon 500
     */
    public CSP_Falcon(int id) {
        super(id, "rio");
        init();
    }

    /**
     * Configures the motor for typical use
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void init() {
        super.configFactoryDefault();
        super.clearStickyFaults();
        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        setEncoder(0.0);
    }

<<<<<<< HEAD
    public void set(double percent) {
        super.set(percent);
    }

=======
    /**
     * Sets the motor to a percentage of its power 
     * @param percent desired percentage of power with range [-1.0, 1.0]
     */
    public void set(double percent) {
        super.set(percent);
    }
    
    /**
     * Sets the motor to a voltage
     * @param voltage desired number of volts
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

<<<<<<< HEAD
=======
    /**
     * Sets the motor to a velocity
     * @param velocity desired velocity
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setVelocity(double velocity) {
        super.set(ControlMode.Velocity, velocity);
    }

<<<<<<< HEAD
=======
    /**
     * Sets the motor to a position
     * @param position desired position
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setPosition(double position) {
        super.set(ControlMode.Position, position);
    }

<<<<<<< HEAD
=======
    /**
     * Sets whether the motor is inverted
     * @param inverted true: inverted, false: not inverted
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setInverted(boolean inverted) {
        super.setInverted(inverted);
    }

<<<<<<< HEAD
    public void setBrake(boolean brake) {
        super.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

=======
    /**
     * Sets whether the motor brakes
     * @param braked true: brake, false: coast
     */
    public void setBrake(boolean braked) {
        super.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Sets the ramp rate of the motor
     * @param rampRate time it takes to ramp up in seconds
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setRampRate(double rampRate) {
        super.configClosedloopRamp(rampRate);
        super.configOpenloopRamp(rampRate);
    }   

<<<<<<< HEAD
=======
    /**
     * Sets the PIDF gains for the motor
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF feedforward gain
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setPIDF(double kP, double kI, double kD, double kF) {
        super.config_kP(0, kP);
        super.config_kI(0, kI);
        super.config_kD(0, kD);
        super.config_kF(0, kF);
    }

<<<<<<< HEAD
=======
    /**
     * Sets the encoder to a desired position
     * @param position desired position to set encoder to
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public void setEncoder(double position) {
        super.setSelectedSensorPosition(position);
    }

<<<<<<< HEAD
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

=======
    /**
     * Sets the conversion ratio
     * @param scalar value to multiply the encoder value by
     */
    public void setScalar(double scalar) {
        super.configSelectedFeedbackCoefficient(scalar);
    }
    
    /**
     * Returns the position of the motor
     * @return position of the motor
     */
    public double getPosition() {
        return (super.getSelectedSensorPosition());
    }

    /**
     * Returns the velocity of the motor
     * @return velocity of the motor
     */
    public double getVelocity() {
        return ((super.getSelectedSensorVelocity() * 10) * 60);
    }

    /**
     * Returns the current of the motor
     * @return current of the motor in amps
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public double getCurrent() {
        return super.getStatorCurrent();
    }

<<<<<<< HEAD
=======
    /**
     * Returns the temperature of the motor
     * @return temperature of the motor in Celcius
     */
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    public double getTemperature() {
        return super.getTemperature();
    }

<<<<<<< HEAD
    public double getCPR() {
        return CPR;
=======
    /**
     * Returns the CAN ID of the motor
     * @return motor CAN ID
     */
    public int getID() {
        return super.getDeviceID();
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
    }
}
