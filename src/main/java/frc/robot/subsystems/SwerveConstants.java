package frc.robot.subsystems;

public class SwerveConstants {
    // Left Forward Swerve Module
    public static final int ID_LEFT_FWD_DRIVE     = 20;
    public static final int ID_LEFT_FWD_STEER     = 21;
    public static final int ID_LEFT_FWD_CANCODER  = 22;
    
    // Right Forward Swerve Module    
    public static final int ID_RIGHT_FWD_DRIVE    = 23;
    public static final int ID_RIGHT_FWD_STEER    = 24;
    public static final int ID_RIGHT_FWD_CANCODER = 25;
    
    // Right Aft Swerve Module    
    public static final int ID_RIGHT_AFT_DRIVE    = 26;
    public static final int ID_RIGHT_AFT_STEER    = 27;
    public static final int ID_RIGHT_AFT_CANCODER = 28;
        
    // Left Aft Swerve Module    
    public static final int ID_LEFT_AFT_DRIVE     = 29;
    public static final int ID_LEFT_AFT_STEER     = 30;
    public static final int ID_LEFT_AFT_CANCODER  = 31;

    public static final double LEFT_FWD_CANCODER_OFFSET  = -0.006836;
    public static final double LEFT_AFT_CANCODER_OFFSET  = -0.607910;
    public static final double RIGHT_FWD_CANCODER_OFFSET = -0.899902;
    public static final double RIGHT_AFT_CANCODER_OFFSET = -0.440674;

    public static final double FRAME_LENGTH = 0.5715 / 2.0; // meters  (22.5 inches)
    public static final double FRAME_WIDTH  = 0.5715 / 2.0; // meters  (22.5 inches)
    public static final double SWERVE_MODULE_FROM_CENTER = Math.sqrt((FRAME_LENGTH * FRAME_LENGTH) + (FRAME_WIDTH * FRAME_WIDTH));

    // SDS Mark 3, 4, and 4i Drive Gear Ratios 
    public static final double SDSMK3_STANDARD = 8.16 / 1.0; 
    public static final double SDSMK3_FAST     = 6.86 / 1.0;
    public static final double SDSMK4_L1       = 8.14 / 1.0;
    public static final double SDSMK4_L2       = 6.75 / 1.0;
    public static final double SDSMK4_L3       = 6.12 / 1.0;
    public static final double SDSMK4_L4       = 5.14 / 1.0;
    public static final double SDSMK4i_L1      = 8.14 / 1.0;
    public static final double SDSMK4i_L2      = 6.75 / 1.0;
    public static final double SDSMK4i_L3      = 6.12 / 1.0;

    // TALONFX DRIVE CONFIG CONSTANTS
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT     = true;
    public static final int     DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int     DRIVE_PEAK_CURRENT_LIMIT       = 60;
    public static final double  DRIVE_PEAK_CURRENT_DURATION    = 0.1;
    public static final double  DRIVE_KP                       = 0.5; //0.5
    public static final double  DRIVE_KI                       = 0.0; //3.0
    public static final double  DRIVE_KD                       = 0.0; //0.0
    public static final double  DRIVE_KF                       = 0.0;
    public static final double  DRIVE_KS                       = 0.32 / 12.0;
    public static final double  DRIVE_KV                       = 1.51 / 12.0;
    public static final double  DRIVE_KA                       = 0.27 / 12.0;
    public static final double  DRIVE_MAX_SPEED                = 2.0; 
    public static final double  DRIVE_TELEOP_MAX_SPEED         = 2.0;
    public static final double  DRIVE_GEAR_RATIO               = SDSMK4_L1;
    public static final double  DRIVE_WHEEL_DIAMETER           = 4.0;
    public static final double  DRIVE_WHEEL_CIRCUMFERENCE      = DRIVE_WHEEL_DIAMETER * Math.PI;

    // TALONFX STEERING CONFIG CONSTANTS
    public static final boolean STEER_ENABLE_CURRENT_LIMIT                = false;
    public static final double  STEER_GEAR_RATIO                          = 12.8 / 1.0;
    public static final int     STEER_CONTINUOUS_CURRENT_LIMIT            = 25;
    public static final int     STEER_PEAK_CURRENT_LIMIT                  = 40;
    public static final double  STEER_PEAK_CURRENT_DURATION               = 0.1;
    public static final double  STEER_KP                                  = 60.0;
    public static final double  STEER_KI                                  = 0.0;
    public static final double  STEER_KD                                  = 0.05;
    public static final double  STEER_KF                                  = 0.0;
    public static final double  STEER_MAX_SPEED                           = 1.0;
    public static final double  TURNING_RADIUS_METERS                     = Math.sqrt(Math.pow(FRAME_LENGTH / 2.0, 2.0) + Math.pow(FRAME_WIDTH / 2.0, 2.0));
    public static final double  STEER_TELEOP_MAX_SPEED_RADIAN_PER_SECOND  = 1.5 / TURNING_RADIUS_METERS;

    public static final double  FARIS_SPEED_MODE_SCALAR  = 0.2; // Pecent. Should be between 0.1 and 1.0
    public static final double  ZERO_SWERVE_TIME_SECONDS = 5.0;
    
}
