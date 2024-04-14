package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.T34XboxController;
import frc.robot.subsystems.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class ControllerDriveCommand extends Command {
    private SwerveDrive m_swerve_drive = null;
    private T34XboxController m_controller = null;
    private double m_driving_speed = 0.0;

    SlewRateLimiter m_x_limiter = null;
    SlewRateLimiter m_y_limiter = null;
    SlewRateLimiter m_r_limiter = null;

    private InstantSource clock = null;
    private Instant m_last_zero = null;

    public ControllerDriveCommand(SwerveDrive drive, T34XboxController controller, InstantSource clock) {
        this.m_swerve_drive = drive;
        this.m_controller = controller;

        this.m_x_limiter = new SlewRateLimiter(1.1);
        this.m_y_limiter = new SlewRateLimiter(1.1);
        this.m_r_limiter = new SlewRateLimiter(2.0);
        
        this.clock = clock;
        this.m_last_zero = clock.instant();

        this.addRequirements(drive);
    }

    @Override
    public void execute() {
        final double x   = this.m_controller.getLeftStickXDB();
        final double y   = this.m_controller.getLeftStickYDB();
        final double rot = this.m_controller.getRightStickXDB();

        if (isInputZero(x, y, rot)) {
            this.m_swerve_drive.stop();

            // When there is no input and when the alloted time has elapsed, rezero the swerve wheels.
            // The alloted time is in seconds and can be set using the ZERO_SWERVE_TIME_SECONDS constexpr
            // located in SwerveContants.h. Suggested value is 5 seconds.
            final Instant now = this.clock.instant();
            final Duration elapsed = Duration.between(this.m_last_zero, now);
            if (elapsed.getSeconds() >= SwerveConstants.ZERO_SWERVE_TIME_SECONDS) {
                this.m_last_zero = now;
                this.m_swerve_drive.resetToAbsolute();
            }  

            return;
        }
        
        double x_speed = Math.copySign(ScaleToRange(-(x * x), 0.0, 1.0, 0.0, SwerveConstants.DRIVE_MAX_SPEED), x);
        double y_speed = Math.copySign(ScaleToRange(-(y * y), 0.0, 1.0, 0.0, SwerveConstants.DRIVE_MAX_SPEED), y);
        double r_speed = Math.copySign(ScaleToRange(-(rot * rot), 0.0, 1.0, 0.0, SwerveConstants.STEER_MAX_SPEED), rot);

        this.m_swerve_drive.drive(new Translation2d(x_speed, y_speed), r_speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_swerve_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static double ScaleToRange(
        final double x, 
        final double in_min, 
        final double in_max, 
        final double out_min, 
        final double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private boolean isInputZero(double x, double y, double r) {
        final double pos_tolerance =  0.00001;
        final double neg_tolerance = -0.00001;

        double combined = x + y + r;
        return combined < pos_tolerance && combined > neg_tolerance;
    }
}
