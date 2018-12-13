package org.frc5687.kachow.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.kachow.robot.OI;
import org.frc5687.kachow.robot.subsystems.DriveTrain;

/**
 * Created by Ben Bernard on 10/23/2018.
 */
public class TeleDrive extends Command {

    private OI _oi;
    private DriveTrain _driveTrain;

    public TeleDrive(OI oi,  DriveTrain driveTrain) {
        _oi = oi;
        _driveTrain = driveTrain;
        requires(driveTrain);
    }

    @Override
    protected void execute() {
        DriveTrain.DriveMode driveMode = _driveTrain.getDriveMode();
        switch (_driveTrain.getDriveMode()) {
            case ARCADE:
                // Get the base speed from the throttle
                double speed = _oi.getDriveSpeed(driveMode);

                // Get the rotation from the tiller
                double rotation = _oi.getDriveRotation(driveMode);

                // Call the drivetrain
                _driveTrain.arcadeDrive(speed, rotation);
                break;

            case TANK:
                double leftSpeed = _oi.getLeftSpeed();
                double rightSpeed = _oi.getRightSpeed();

                _driveTrain.tankDrive(leftSpeed, rightSpeed);
                break;
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
