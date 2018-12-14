package org.frc5687.kachow.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.kachow.robot.Constants;
import org.frc5687.kachow.robot.Robot;
import org.frc5687.kachow.robot.RobotMap;
import org.frc5687.kachow.robot.commands.AllDrive;
import org.frc5687.kachow.robot.utils.Helpers;

import static org.frc5687.kachow.robot.utils.Helpers.limit;

/**
 * Created by Ben Bernard on 6/4/2018.
 */
public class DriveTrain extends Subsystem  implements PIDSource {
    // Add the objects for the motor controllers

    VictorSP _leftFront;
    VictorSP _leftRear;

    VictorSP _rightFront;
    VictorSP _rightRear;

    private Robot _robot;
    private DriveMode _driveMode = DriveMode.CHEESY_ARCADE;
    public AHRS _imu;

    public DriveTrain(Robot robot) {
        _robot = robot;
        _imu = robot.getIMU();

        // Motor Initialization
        _leftFront = new VictorSP(RobotMap.PWM.LEFT_FRONT_VICTOR);
        _leftRear = new VictorSP(RobotMap.PWM.LEFT_REAR_VICTOR);

        _rightFront = new VictorSP(RobotMap.PWM.RIGHT_FRONT_VICTOR);
        _rightRear = new VictorSP (RobotMap.PWM.RIGHT_REAR_VICTOR);

        // Setup followers to follow their master

        // Setup motors

        _leftFront.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
        _leftRear.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
        _rightFront.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
        _rightRear.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);

        // Configure the encoders
        resetDriveEncoders();


    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new AllDrive(this, _robot.getOI()));
    }

    public void setPower(double leftSpeed, double rightSpeed) {
        setPower(leftSpeed, rightSpeed, false);
    }
    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
        try {
            _leftFront.set(leftSpeed);
            _rightFront.set(rightSpeed);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.setPower exception: " + e.toString(), false);
        }
        SmartDashboard.putNumber("DriveTrain/PowerRight", rightSpeed);
        SmartDashboard.putNumber("DriveTrain/PowerLeft", leftSpeed);
    }


    public void resetDriveEncoders() {
        try {
            // _leftFront.setSelectedSensorPosition(0,0,0);
            // _rightFront.setSelectedSensorPosition(0, 0, 0);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.resetDriveEncoders exception. I suppose this is really bad. : " + e.toString(), false);
        }
    }


    public void arcadeDrive(double speed, double rotation) {
        SmartDashboard.putNumber("DriveTrain/Speed", speed);
        SmartDashboard.putNumber("DriveTrain/Rotation", rotation);

        speed = limit(speed);

        rotation = limit(rotation);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        speed = Math.copySign(speed * speed, speed);
        rotation = Math.copySign(rotation * rotation, rotation);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed >= 0.0) {
            // First quadrant, else second quadrant
            if (rotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (rotation >= 0.0) {
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }

        setPower(leftMotorOutput, rightMotorOutput);
    }

    public void cheesyDrive(double speed, double rotation) {
        SmartDashboard.putNumber("DriveTrain/Speed", speed);
        SmartDashboard.putNumber("DriveTrain/Rotation", rotation);

        speed = limit(speed);
        Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        speed = Math.copySign(speed * speed, speed);
        rotation = Helpers.applySensitivityFactor(rotation,  gear == Shifter.Gear.HIGH ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed==0.0) {
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
        } else {
            double delta = rotation * Math.abs(speed);
            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput));
    }


    public void tankDrive(double leftSpeed, double rightSpeed, boolean overrideCaps) {
        SmartDashboard.putNumber("DriveTrain/LeftSpeed", leftSpeed);
        SmartDashboard.putNumber("DriveTrain/RightSpeed", rightSpeed);
        double leftMotorOutput = leftSpeed;
        double rightMotorOutput = rightSpeed;



        setPower(leftMotorOutput, rightMotorOutput);
    }

    public float getYaw() {
        return _imu.getYaw();
    }

    /**
     * Get the number of ticks since the last reset
     * @return
     */
    public long getLeftTicks() {
        return 0; // _leftFront.getSelectedSensorPosition(0);
    }
    public long getRightTicks() {
        return 0; //_rightFront.getSelectedSensorPosition(0);
    }

    /**
     * The left distance in Inches since the last reset.
     * @return
     */
    public double getLeftDistance() {
        return getLeftTicks() * Constants.Encoders.INCHES_PER_PULSE;
    }
    public double getRightDistance() {
        return getRightTicks() * Constants.Encoders.INCHES_PER_PULSE;
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




    /*
    public void curvaturerive(double speed, double rotation, boolean isQuickTurn) {

        speed = limit(speed);

        rotation = limit(rotation);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(speed) < m_quickStopThreshold) {
                m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                        + m_quickStopAlpha * limit(zRotation) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        m_leftMotor.set(leftMotorOutput * m_maxOutput);
        m_rightMotor.set(-rightMotorOutput * m_maxOutput);

        m_safetyHelper.feed();
    }
*/

    public void setCurrentLimiting(int amps) {
        //_leftFront.configContinuousCurrentLimit(amps, 0);
        //_leftFront.configPeakCurrentLimit(0, 0);
        //_rightFront.configContinuousCurrentLimit(amps, 0);
        //_rightFront.configPeakCurrentLimit(0, 0);
    }


    public double getLeftSpeed() {
        return 0; // _leftFront.getMotorOutputPercent() / Constants.DriveTrain.HIGH_POW;
    }

    public double getRightSpeed() {
        return 0; //_rightFront.getMotorOutputPercent() / Constants.DriveTrain.HIGH_POW;
    }

    public double getLeftRate() {
        return 0; // _leftFront.getSelectedSensorVelocity(0);
    }

    public double getRightRate() {
        return 0; // _rightFront.getSelectedSensorVelocity(0);
    }

    public DriveMode getDriveMode() { return _driveMode; }

    @Override
    public double pidGet() {
        return getDistance();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    public void setDriveMode(DriveMode driveMode) { _driveMode = driveMode; }

    public void enableBrakeMode() {
        try {
//            _leftFront.setNeutralMode(NeutralMode.Brake);
//            _leftRear.setNeutralMode(NeutralMode.Brake);
//
//            _rightFront.setNeutralMode(NeutralMode.Brake);
//            _rightRear.setNeutralMode(NeutralMode.Brake);

        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.enableBrakeMode exception: " + e.toString(), false);
        }
        SmartDashboard.putString("DriveTrain/neutralMode", "Brake");
    }

    public void enableCoastMode() {
        try {
//            _leftFront.setNeutralMode(NeutralMode.Coast);
//            _leftRear.setNeutralMode(NeutralMode.Coast);
//            _rightFront.setNeutralMode(NeutralMode.Coast);
//            _rightRear.setNeutralMode(NeutralMode.Coast);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.enableCoastMode exception: " + e.toString(), false);
        }
        SmartDashboard.putString("DriveTrain/neutralMode", "Coast");
    }



    public enum DriveMode {
        TANK(0),
        ARCADE(1),
        CHEESY_ARCADE(2);

        private int _value;

        DriveMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    public void updateDashboard() {
        SmartDashboard.putNumber("DriveTrain/LeftDistance", getLeftDistance());
        SmartDashboard.putNumber("DriveTrain/RIghtDistance", getRightDistance());
        SmartDashboard.putNumber("DriveTrain/LeftRate", getLeftRate());
        SmartDashboard.putNumber("DriveTrain/RightRate", getRightRate());
        SmartDashboard.putNumber("DriveTrain/LeftSpeed", getLeftSpeed());
        SmartDashboard.putNumber("DriveTrain/RightSpeed", getRightSpeed());
        SmartDashboard.putNumber("DriveTrain/Yaw", _imu.getYaw());
    }

}
