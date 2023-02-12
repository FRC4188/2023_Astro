package frc.robot.AidenLib;

/** A class which keeps track of time. */
public class Timer {
    private long lastTime, startTime;

    /**
     * Constructs a {@link Timer} object.
     */
    public Timer() {
        this.lastTime = System.currentTimeMillis();
        this.startTime = System.currentTimeMillis();
    };

    /**
     * @return The length of time since the last call of this method.
     */
    public double getDT() {
        long time = System.currentTimeMillis();
        double dt = (time - lastTime) / 1000.0;
        lastTime = time;
        return dt;
    }

    /**
     * Starts/restarts a stopwatch.
     */
    public void start() {
        this.startTime = System.currentTimeMillis();
    }

    /**
     * @return The current time into the stopwatch.
     */
    public double getDuration() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }
}
