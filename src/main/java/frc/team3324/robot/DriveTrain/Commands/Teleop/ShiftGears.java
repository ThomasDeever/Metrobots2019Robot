package frc.team3324.robot.DriveTrain.Commands.Teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3324.robot.util.OI;
import frc.team3324.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShiftGears extends Command {

    private boolean gearShifterStatus = false;

    public ShiftGears() {
        super("ShiftGears");
        requires(Robot.mDriveTrain);
    }

    protected void initialize() {}

    protected void execute() {
        SmartDashboard.putBoolean("SHIFTING", true);
        if (OI.primaryController.getAButton()) {
            if (gearShifterStatus) {
                Robot.mDriveTrain.setHighGear();
                gearShifterStatus = !gearShifterStatus;
            } else {
                Robot.mDriveTrain.setLowGear();
                gearShifterStatus = !gearShifterStatus;
            }
        }
    }

    protected boolean isFinished() { return true; }

    protected void end() {}

    protected void interrupted() {}
}