package org.frc5687.kachow.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc5687.kachow.robot.Constants;
import org.frc5687.kachow.robot.OI;
import org.frc5687.kachow.robot.RobotMap;
import org.frc5687.kachow.robot.commands.TeleDrive;

import static org.frc5687.kachow.robot.utils.Helpers.limit;

public class DriveTrain extends Subsystem {

    // Subsystem references here
    private OI _oi;
    private AHRS _imu;

    // TODO: Add member variables for motor controllers here
    private TalonSRX _leftMaster;
    private TalonSRX _rightMaster;
    private VictorSPX _leftFollower;
    private VictorSPX _rightFollower;

    private DriveMode _driveMode = DriveMode.ARCADE;

    public DriveTrain(OI oi, AHRS imu) {
        _oi = oi;
        _imu = imu;

        // TODO: Initialize motor controllers here
        _leftMaster = new TalonSRX(RobotMap.CAN.LEFT_MASTER);
        _leftFollower = new VictorSPX(RobotMap.CAN.LEFT_FOLLOWER);

        _rightMaster = new TalonSRX(RobotMap.CAN.RIGHT_MASTER);
        _rightFollower = new VictorSPX(RobotMap.CAN.RIGHT_FOLLOWER);

        _leftFollower.follow(_leftMaster);
        _rightFollower.follow(_rightMaster);

        _leftMaster.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);
        _leftFollower.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);

        _rightMaster.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);
        _rightFollower.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);

        _leftMaster.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);
        _leftFollower.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);

        _rightMaster.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);
        _rightFollower.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);

        _leftMaster.configNominalOutputForward(0.0, 0);
        _leftFollower.configNominalOutputForward(0.0, 0);

        _rightMaster.configNominalOutputForward(0.0, 0);
        _rightFollower.configNominalOutputForward(0.0, 0);

        _leftMaster.configNominalOutputReverse(0.0, 0);
        _leftFollower.configNominalOutputReverse(0.0, 0);

        _rightMaster.configNominalOutputReverse(0.0, 0);
        _rightFollower.configNominalOutputReverse(0.0, 0);

        _leftMaster.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
        _leftFollower.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);

        _rightMaster.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
        _rightFollower.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);

        _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        resetDriveEncoders();

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TeleDrive(_oi, this));
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        // TODO: Implement tank drive logic here!
    }

    public void arcadeDrive(double speed, double rotation) {



        speed = limit(speed);

        rotation = limit(rotation);

        speed = Math.copySign(speed * speed, speed);
        rotation = Math.copySign(rotation * rotation, rotation);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed >= 0.0) {

            if (rotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                leftMotorOutput = speed - rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            if (rotation >= 0.0) {
                leftMotorOutput = speed - rotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }
        // Currently don't have "setPower" method
        // setPower(leftMotorOutput, rightMotorOutput);
    }


    /**
     * Get the number of ticks since the last reset
     * @return
     */
    public long getLeftTicks() {
        return _leftMaster.getSelectedSensorPosition(0);
    }
    public long getRightTicks() {
        return _rightMaster.getSelectedSensorPosition(0);
    }

    /**
     * The left distance in Inches since the last reset.
     * @return
     */
    public double getLeftDistance() {
        return getLeftTicks() * Constants.DriveTrain.Encoders.INCHES_PER_PULSE;
    }
    public double getRightDistance() {
        return getRightTicks() * Constants.DriveTrain.Encoders.INCHES_PER_PULSE;
    }


    public void resetDriveEncoders() {
        try {
            _leftMaster.setSelectedSensorPosition(0,0,0);
            _rightMaster.setSelectedSensorPosition(0, 0, 0);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.resetDriveEncoders exception. : " + e.toString(), false);
        }
    }

    /**
     * @return average of leftDistance and rightDistance
     */
    public double getDistance() {
        if (Math.abs(getRightTicks())<10) {
            return getLeftDistance();
        }
        if (Math.abs(getLeftTicks())<10) {
            return getRightDistance();
        }
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public DriveMode getDriveMode() { return _driveMode; }

    public void setDriveMode(DriveMode driveMode) { _driveMode = driveMode; }


    public enum DriveMode {
        TANK(0),
        ARCADE(1),
        ARC(2);

        private int _value;

        DriveMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

}

