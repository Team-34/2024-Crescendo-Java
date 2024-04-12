package frc.robot;

public final class Maths {
    public static double clamp(final double value, final double min, final double max) {
        assert min < max;

        return Math.min(max, Math.max(min, value));
    }
    
    public static double log2(final double x) {
        return Math.log(x) / Math.log(2);
    }
}
