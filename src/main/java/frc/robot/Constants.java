// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final boolean INVERT_GYRO = true;

    public static final double NEO550_RES = 42;
    public static final double INTAKE_GEAR_RATIO = 203.636364;
    public static final double ARM_ENC_CONVERSION_FACTOR = 360.0 / (NEO550_RES * INTAKE_GEAR_RATIO);
    public static final double CLIMBER_UNITS_TO_INCHES_FACTOR = 1;
    public static final double ARM_DEG_SCALAR = 0.02756;
    public static final double SHOOTER_DEG_SCALAR = 0.0116;
    public static final double LIMELIGHT_DEGREE_SCALAR = 23.188 / 20.25; // 21.1726 / 22.5;
    public static final double SHOOTER_OFFSET_ANGLE_DEG = 59;

    public static final String LIMELIGHT_TABLE_NAME = "";

    public static final int POV_UP    = 0;
    public static final int POV_RIGHT = 90;
    public static final int POV_DOWN  = 180;
    public static final int POV_LEFT  = 270;
}
