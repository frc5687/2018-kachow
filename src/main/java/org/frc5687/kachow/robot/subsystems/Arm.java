package org.frc5687.kachow.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc5687.kachow.robot.Robot;
import edu.wpi.first.wpilibj.VictorSP;
import org.frc5687.kachow.robot.RobotMap;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.frc5687.kachow.robot.commands.DriveArm;
import org.frc5687.kachow.robot.utils.AnglePotentiometer;
import org.frc5687.kachow.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Arm extends Subsystem {
    private Robot _robot;
    private VictorSP _motor;
    private AnglePotentiometer _pot;
    private DigitalInput _frontLimit;
    private DigitalInput _rearLimit;
    private boolean _atFrontLimit = false;
    private boolean _atRearLimit = false;
    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new DriveArm(this, _robot.getOI()));
    }
    public void drive(double speed){
        if (speed > 0 && atFrontLimit()) {
            speed = 0;
        } else if (speed < 0 && atRearLimit()) {
            speed = 0;
        }

        _motor.set((Constants.Arm.MOTOR_INVERTED ? -1 : 1) * speed);
        if (!_frontLimit.get()) {
            _pot.restTop();
            SmartDashboard.putNumber("Arm/TopPot",_pot.getRaw());
        }
        if (!_rearLimit.get()) {
            _pot.restBottom();
            SmartDashboard.putNumber("Arm/BottomPot",_pot.getRaw());
        }
    }

    public boolean atFrontLimit() {
        return /*_atFrontLimit || */  !_frontLimit.get() ||  getAngle() >= Constants.Arm.ANGLE_MAX;
    }

    public boolean atRearLimit() {
        return /*_atRearLimit || */ !_rearLimit.get() ||  getAngle() <= Constants.Arm.ANGLE_MIN;
    }

    public Arm(Robot robot) {
        _robot = robot;
        _motor = new VictorSP(RobotMap.PWM.ARM_MOTOR);
        _pot = new AnglePotentiometer(RobotMap.Analog.ARM_POTENTIOMETER, Constants.Arm.ANGLE_MIN,
                Constants.Arm.POT_MIN, Constants.Arm.ANGLE_MAX,  Constants.Arm.POT_MAX);
        _frontLimit = new DigitalInput(RobotMap.DIO.ARM_FRONT_LIMIT);
        _rearLimit = new DigitalInput(RobotMap.DIO.ARM_REAR_LIMIT);
    }

    public double getPot() {
        return _pot.get();
    }

    public double getAngle() {
        return getPot();
    }
}
