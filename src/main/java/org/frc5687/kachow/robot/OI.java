package org.frc5687.kachow.robot;

import edu.wpi.first.wpilibj.Joystick;
import org.frc5687.kachow.robot.subsystems.DriveTrain;
import org.frc5687.kachow.robot.subsystems.Shifter;
import org.frc5687.kachow.robot.utils.Gamepad;
import static org.frc5687.kachow.robot.utils.Helpers.*;
import org.frc5687.kachow.robot.Constants;



public class OI {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;


    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
    }

    private Shifter.Gear _gear = Shifter.Gear.LOW;


    public double getLeftSpeed() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        double sensitivity = _gear == Shifter.Gear.LOW ? Constants.DriveTrain.SENSITIVITY_LOW_GEAR : Constants.DriveTrain.SENSITIVITY_HIGH_GEAR;
        speed = applyDeadband(speed, sensitivity);


        return speed;

    }

    public double getRightSpeed() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        double sensitivity = _gear == Shifter.Gear.LOW ? Constants.DriveTrain.SENSITIVITY_LOW_GEAR : Constants.DriveTrain.SENSITIVITY_HIGH_GEAR;
        speed = applySensitivityFactor(speed, sensitivity);

        return speed;
    }
    public double getArmSpeed(){
        double speed = -getSpeedFromAxis(_operatorGamepad, 5) * Constants.Arm.SPEED_MAX;
        speed = applyDeadband(speed, Constants.Arm.DEADBAND);
        return applySensitivityFactor(speed, Constants.Arm.SENSITIVITY);
    }

    public double getDriveSpeed(DriveTrain.DriveMode driveMode) {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return applySensitivityFactor(speed, Constants.DriveTrain.SENSITIVITY_LOW_GEAR);

    }

    public double getDriveRotation(DriveTrain.DriveMode driveMode) {
        double speed = driveMode == DriveTrain.DriveMode.ARCADE ?
                getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber())
                : getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return applySensitivityFactor(speed, Constants.DriveTrain.ROTATION_SENSITIVITY);

    }
    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

}
