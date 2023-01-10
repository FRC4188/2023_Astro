package frc.robot.utils.math;

public class Integral {

    private double val;
    private double lastTime = System.currentTimeMillis();
    
    public Integral(double c) {
        val = c;
    }

    public void sample(double value) {
        val += value * (System.currentTimeMillis() - lastTime) / 1000.0;
        lastTime = System.currentTimeMillis();
    }

    public double getValue() {
        return val;
    }
}
