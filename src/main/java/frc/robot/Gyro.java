package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class Gyro extends AHRS {
    private static final Gyro instance = new Gyro();

    private Gyro() {
        super(SPI.Port.kMXP);
    }

    public static Gyro get() {
        return instance;
    }

    public void ZeroYaw() {
        this.reset();
    }
}
