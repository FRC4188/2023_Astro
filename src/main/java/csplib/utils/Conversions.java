package csplib.utils;

public final class Conversions {
    /**
     * Converts angles with range [-180, 180] to [0, 360]
     * @param input 
     * @return the angle between [0, 360]
     */
    public static double degreesSignedToUnsigned(double input) {
        return (-input + 360.0) % 360.0 - 180.0;
    }
}
