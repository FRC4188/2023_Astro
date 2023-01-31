package csplib.motors;

public interface CSP_Motor {

    public void setInverted(boolean inverted);

    public void init();

    public void setBrake(boolean braking);

    public void setRampRate(double rampRate);
    
    public void set(double percent);

    public void setVoltage(double volts);

    public void setEncoder(double position);

    public void setPIDF(double kP, double kI, double kD, double kF);

<<<<<<< HEAD
    public void setPositionScalar(double scalar);

    public void setVelocityScalar(double scalar);
=======
    public void setScalar(double scalar);
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf

    public void setPosition(double position);
    
    public void setVelocity(double velocity);

    public double getVelocity();

    public double getPosition();

    public double getTemperature();

    public double getCurrent();
<<<<<<< HEAD
=======

    public int getID();
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
}