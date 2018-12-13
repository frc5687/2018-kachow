package org.frc5687.kachow.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.kachow.robot.OI;
import org.frc5687.kachow.robot.subsystems.Arm;

public class DriveArm extends Command {
    private OI _oi;
    private Arm _arm;
    public DriveArm(OI oi, Arm arm){
        _oi = oi;
        _arm = arm;
        requires(arm);
    }
    @Override
    protected void execute(){
        double speed = _oi.getArmSpeed();
        _arm.drive(speed);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
