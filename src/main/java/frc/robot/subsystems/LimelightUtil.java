package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Maths;
import frc.robot.TrajMath;

public class LimelightUtil {
    public enum TargetMode
    {
        kNone, //(-1),
        kSpeaker,
        kAmp,
        kTrap
    }

    // private static class SwerveSpeeds
    // {
    //     public double x;
    //     public double y;
    //     public double r;

    //     public SwerveSpeeds(final double)
    // }

    public static class SwerveSpeeds {
        public double x;
        public double y;
        public double r;

        public SwerveSpeeds(final double x, final double y, final double r) {
            this.x = x;
            this.y = y;
            this.r = r;
        }
    }

    private PIDController m_limelight_swerve_pid;

    private NetworkTable m_table;


    private double m_tx = LimelightHelpers.getTX(Constants.LIMELIGHT_TABLE_NAME);
    private double m_ty = LimelightHelpers.getTY(Constants.LIMELIGHT_TABLE_NAME);

    private double m_heading_error;
    private double m_kp;
    private double m_min_command;

    private double m_steering_adjust;
    private double m_drive_x;
    private double m_drive_y;

    private double m_current_id;
    private double m_target_id;

    private TargetMode m_target_mode;

    private void adjustSteering() {
        if (Math.abs(this.m_heading_error) > 1.0) {
            this.m_steering_adjust = this.m_heading_error < 0.0 ?
                this.m_kp * this.m_heading_error + this.m_min_command :
                this.m_kp * this.m_heading_error - this.m_min_command;
        }
    }

    public TrajMath m_math_handler;
    public SwerveSpeeds m_swerve_drive_speeds;

    public LimelightUtil(final TrajMath math_handler) {
        this(math_handler, TargetMode.kSpeaker);
    }

    public LimelightUtil(final TrajMath math_handler, final TargetMode target_mode) {
        this.m_limelight_swerve_pid = new PIDController(0.1, 0.0, 0.1);
        this.m_table = LimelightHelpers.getLimelightNTTable(Constants.LIMELIGHT_TABLE_NAME);
        this.m_tx = LimelightHelpers.getTX(Constants.LIMELIGHT_TABLE_NAME);
        this.m_ty = LimelightHelpers.getTY(Constants.LIMELIGHT_TABLE_NAME);
        this.m_heading_error = -m_tx;
        this.m_kp = -0.1;
        this.m_min_command = 0.5;
        this.m_steering_adjust = 0.0;
        this.m_drive_x = 0.0;
        this.m_drive_y = 0.0;
        this.m_swerve_drive_speeds = new SwerveSpeeds(0.0, 0.0, 0.0);
        this.m_math_handler = math_handler;
        this.m_current_id = -1.0;
        this.m_target_id = -1.0;
        this.m_target_mode = target_mode;
    }

    public void Init() {

    }

    public void periodic() {
        this.m_table = LimelightHelpers.getLimelightNTTable(Constants.LIMELIGHT_TABLE_NAME);
        this.m_tx = LimelightHelpers.getTX(Constants.LIMELIGHT_TABLE_NAME);
        this.m_ty = LimelightHelpers.getTY(Constants.LIMELIGHT_TABLE_NAME);
        this.m_current_id = LimelightHelpers.getFiducialID(Constants.LIMELIGHT_TABLE_NAME);
        this.m_math_handler.periodic();
    
        switch (this.m_target_mode)
        {
            case kAmp: {
                this.m_math_handler.setTargetHeightMeters(0.864);
                this.m_math_handler.setAprilTagHeightMeters(1.45);
                
                final var alliance = DriverStation.getAlliance();
                if (alliance.isEmpty()) {
                    this.m_target_id = -1.0;
                } else {
                    this.m_target_id =
                        ((this.m_current_id == 5 && alliance.get() == DriverStation.Alliance.Red) ||
                         (this.m_current_id == 6 && alliance.get() == DriverStation.Alliance.Blue))
                        ? this.m_current_id
                        : -1.0;
                }
                break;
            }
            case kSpeaker: {
                this.m_math_handler.setTargetHeightMeters(1.9815);
                this.m_math_handler.setAprilTagHeightMeters(1.435);
                
                final var alliance = DriverStation.getAlliance();
                if (alliance.isEmpty()) {
                    this.m_target_id = -1.0;
                } else {
                    this.m_target_id =
                        ((this.m_current_id == 4 && alliance.get() == DriverStation.Alliance.Red) ||
                         (this.m_current_id == 7 && alliance.get() == DriverStation.Alliance.Blue))
                        ? this.m_current_id
                        : -1.0;
                }
                break;
            }

            default:
                break;
        }
    
        this.m_heading_error = Maths.clamp(-this.m_tx, -1.0, 1.0);
        this.m_drive_x = Maths.clamp((this.m_tx / 25.0), -1.0, 1.0);
        this.m_drive_y = this.m_math_handler.isInRange() ? 0.0 : 0.5;
    
        //this.m_drive_x = std.copysign(ScaleToRange(-(this.m_drive_x * this.m_drive_x), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), this.m_drive_x);
        //this.m_drive_y = std.copysign(ScaleToRange(-(this.m_drive_y * this.m_drive_y), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), this.m_drive_y);
        //this.m_steering_adjust = std.copysign(ScaleToRange(-(this.m_steering_adjust * this.m_steering_adjust), 0.0, 1.0, 0.0, STEER_MAX_SPEED), this.m_steering_adjust);
    
        this.m_swerve_drive_speeds.x = this.m_drive_x;
        this.m_swerve_drive_speeds.y = this.m_drive_y;
        this.m_swerve_drive_speeds.r = this.m_steering_adjust;
    }

    //inline void SetTargetMode(TargetMode mode) { m_target_mode = mode; }

    public void targetSpeaker() {
        this.m_target_mode = TargetMode.kSpeaker;
    }

    public void targetAmp() {
        this.m_target_mode = TargetMode.kAmp;
    }
    
    public void targetTrap() {
        this.m_target_mode = TargetMode.kTrap;
    }

    public double getTargetID() {
        return this.m_target_id;
    }
}
