package csplib.utils;

import edu.wpi.first.wpilibj.Preferences;

public class Saver {

    private static boolean contains(String key) {
        return Preferences.containsKey(key);
    }

    public static void setDouble(String key, double value) {
        if (!contains(key)) Preferences.setDouble(key, value);
    }
} 
