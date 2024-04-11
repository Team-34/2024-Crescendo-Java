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

    private InstantSource clock = null;
    private Instant m_last_zero;

    private SlewRateLimiter m_x_speed_limiter   = null;
    private SlewRateLimiter m_y_speed_limiter   = null;
    private SlewRateLimiter m_rot_speed_limiter = null;

    public ControllerDriveCommand(SwerveDrive drive, T34XboxController controller, InstantSource clock) {
        this.m_swerve_drive = drive;
        this.m_controller = controller;

        this.addRequirements(drive);

        this.m_x_speed_limiter = new SlewRateLimiter(0.2);
        this.m_y_speed_limiter = new SlewRateLimiter(0.2);
        this.m_rot_speed_limiter = new SlewRateLimiter(0.2);
        
        this.clock = clock;
        this.m_last_zero = clock.instant();
    }

    @Override
    public void initialize() {
        final double x    = this.m_controller.getLeftStickXDB();
        final double y    = this.m_controller.getLeftStickYDB();
        final double rot  = this.m_controller.getRightStickXDB();

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
        
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final double x_speed = -m_x_speed_limiter.calculate(x);

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final double y_speed = -m_y_speed_limiter.calculate(y);

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final double rot_speed = -m_rot_speed_limiter.calculate(rot);

        // double x_speed = std::copysign(ScaleToRange(-(x * x), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), x);
        // double y_speed = std::copysign(ScaleToRange(-(y * y), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), y);
        // double r_speed = std::copysign(ScaleToRange(-(rot * rot), 0.0, 1.0, 0.0, STEER_MAX_SPEED), rot);

        this.m_swerve_drive.drive(new Translation2d(x_speed, y_speed), rot_speed);

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
        final double out_max
    ) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private boolean isInputZero(double x, double y, double r) {
        final double pos_tolerance =  0.00001;
        final double neg_tolerance = -0.00001;

        double combined = x + y + r;
        return combined < pos_tolerance && combined > neg_tolerance;
    }
}
