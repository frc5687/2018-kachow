package org.frc5687.kachow.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc5687.kachow.robot.Constants;
import org.frc5687.kachow.robot.Robot;
import org.frc5687.kachow.robot.RobotMap;

public class Shifter extends Subsystem {
    private Robot _robot;

    private DoubleSolenoid shifterSolenoid;
    private Compressor compressor;
    private long waitPeriodEndTime = 0;

    public Shifter(Robot robot) {
        _robot = robot;
        shifterSolenoid = new DoubleSolenoid(RobotMap.PCM.SHIFTER_HIGH, RobotMap.PCM.SHIFTER_LOW);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void shift(Gear gear, boolean auto) {
        shifterSolenoid.set(gear.getSolenoidValue());
        waitPeriodEndTime = System.currentTimeMillis() + (auto ? Constants.Shifter.AUTO_WAIT_PERIOD : Constants.Shifter.MANUAL_WAIT_PERIOD);

    }

    public boolean waitPeriodElapsed() { return System.currentTimeMillis() > waitPeriodEndTime;}

    public Gear getGear() {
        DoubleSolenoid.Value current = shifterSolenoid.get();
        if (current==Gear.HIGH.getSolenoidValue()) {
            return Gear.HIGH;
        } else if (current == Gear.LOW.getSolenoidValue()) {
            return Gear.LOW;
        }
        return Gear.UNKNOWN;
    }
    public enum Gear {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        HIGH(DoubleSolenoid.Value.kReverse),
        LOW(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;

        Gear(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }

    }


}
