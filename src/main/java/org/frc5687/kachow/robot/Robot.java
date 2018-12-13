package org.frc5687.kachow.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.frc5687.kachow.robot.subsystems.DriveTrain;

/**
 * Created by Ben Bernard on 10/23/2018.
 */
public class Robot extends TimedRobot {
    private AHRS _imu;
    private DriveTrain _driveTrain;
    private OI _oi;

    @Override
    public void robotInit() {
        setPeriod(1 / Constants.CYCLES_PER_SECOND);
        LiveWindow.disableAllTelemetry();

        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);
        _oi = new OI();
        _driveTrain = new DriveTrain(_oi, _imu);
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        updateDashboard();
    }


    public void updateDashboard() {
    }

}
