package frc.robot.AidenLib.math;

import java.util.ArrayList;
import java.util.LinkedList;

/**
 * Fuses data weighted on the standard deviation in the data from the "true" value.
 */
public class WeightedFusion {

    /**
     * This method applies the Kalman filter algorithm to a series of measurements.
     * @param estimates A {@link Iterable} of {@link Data} objects which represent the series of measurements.
     * @return The estimate of the filter.
     */
    public static Data calculate(Iterable<Data> estimates) {

        double xnumerator = 0.0;
        double denominator = 0.0;

        for (Data est : estimates) {
            xnumerator += Math.pow(est.stDev, -2.0) * est.value;
            denominator += Math.pow(est.stDev, -2.0);
        }

        return new Data(xnumerator / denominator, Math.sqrt(denominator) / denominator);
    }

    /**
     * This method applies the Kalman filter algorithm to a parametric series of measurements.
     * @param estimates A {@link Iterable} of {@link Iterable} objects for each parameter containing {@link Data} objects which represent that parameter's measurements.
     * @return The estimates of the filter.
     */
    public static ArrayList<Data> calculateParameterized(Iterable<Iterable<Data>> estimates) {
        LinkedList<Data> result = new LinkedList<>();

        for (Iterable<Data> param : estimates) {
            result.add(calculate(param));
        }

        return new ArrayList<>(result);
    }
}
