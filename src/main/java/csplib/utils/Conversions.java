package csplib.utils;

public final class Conversions {
  /**
   * Converts angles with range [-180, 180] to [0, 360]
   *
   * @param input
   * @return the angle between [0, 360]
   */
  public static double degreesSignedToUnsigned(double input) {
    return (input < 0.0) ? input + 360 : input;
  }

  /**
   * Converts angles with range [0, 360] to [-180, 180]
   *
   * @param input
   * @return the angle between [-180, 180]
   */
  public static double degreesUnsignedToSigned(double input) {
    return (input + 180) % 360 - 180;
  }
}
