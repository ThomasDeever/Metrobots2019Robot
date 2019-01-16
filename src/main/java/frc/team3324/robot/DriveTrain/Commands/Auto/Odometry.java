package frc.team3324.robot.drivetrain.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3324.robot.Robot;

public class Odometry extends Command {

    private double x;
    private double y;
    private double theta;
    double xPrime;
    double yPrime;
    double phi;
    double thetaPrime;
    double middleEncoder;
    double lEncoder;
    double rEncoder;

    //Nonlinear Control
    double turnCmd;
    double velCmd;
    double turnDesire;
    double velDesire;
    double gamma;
    double kOne;
    double kTwo;
    double kThree;
    double thetaDesire;
    double xDesire;
    double yDesire;

    public Odometry(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.driveTrain.clearEncoder();
        Robot.driveTrain.clearGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        lEncoder = Robot.driveTrain.getLeftEncoder();
        rEncoder = Robot.driveTrain.getRightEncoder();
        middleEncoder = (lEncoder + rEncoder)/2;
        phi = Robot.driveTrain
                .getYaw();

        xPrime = x + middleEncoder * Math.cos(theta);
        yPrime = y + middleEncoder * Math.sin(theta);
        thetaPrime = theta + phi;

        //Nonlinear Control
        velCmd = velDesire * Math.cos(thetaDesire-thetaPrime) + kOne(velDesire,turnDesire) * Math.cos(thetaPrime) + (yDesire - yPrime) * Math.sin(thetaPrime);
        turnCmd = turnDesire + kOne(velDesire,turnDesire) * (thetaDesire-thetaPrime) + (kTwo * velDesire * Math.sin(thetaDesire-thetaPrime) / (thetaDesire-thetaPrime)) * (Math.cos(thetaPrime) * (yDesire - yPrime) - (xDesire - xPrime) * Math.sin(thetaPrime));
    }

    protected boolean isFinished() {}

    // Called once after isFinished returns true
    protected void end() {}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {}

}


