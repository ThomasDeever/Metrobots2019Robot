package frc.team3324.robot.DriveTrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team3324.robot.util.Constants;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.kauailabs.navx.frc.AHRS;
import frc.team3324.robot.DriveTrain.Commands.Teleop.Drive;

// Identify Drivetrain as a subsystem (class)
public class DriveTrain extends Subsystem {
    private ShuffleboardTab sensorTab = Shuffleboard.getTab("Encoder Values");
    private NetworkTableEntry leftDistance = sensorTab.add("Left Encoder Distance", 0).withPosition(0,0).getEntry();
    private NetworkTableEntry rightDistance = sensorTab.add("Right Encoder Distance", 0).withPosition(1,0).getEntry();
    private NetworkTableEntry rightRaw = sensorTab.add("Right Encoder Raw", 0).withPosition(2, 0).getEntry();
    private NetworkTableEntry leftRaw = sensorTab.add("Left Encoder Raw", 0).withPosition(3,0).getEntry();
    private NetworkTableEntry rightRate = sensorTab.add("Right Encoder Rate", 0).withPosition(4,0).getEntry();
    private NetworkTableEntry leftRate = sensorTab.add("Left Encoder Rate", 0).withPosition(5,0).withWidget(BuiltInWidgets.kGraph).getEntry();

    private NetworkTableEntry leftDistanceGraph = sensorTab.add("Left Encoder Distance Graph", 0).withPosition(0,1).withWidget(BuiltInWidgets.kGraph).getEntry();
    private NetworkTableEntry rightDistanceGraph = sensorTab.add("Right Encoder Distance Graph", 0).withPosition(1,1).withWidget(BuiltInWidgets.kGraph).getEntry();
    private NetworkTableEntry rightRawGraph = sensorTab.add("Right Encoder Raw Graph", 0).withPosition(2, 1).withWidget(BuiltInWidgets.kGraph).getEntry();
    private NetworkTableEntry leftRawGraph = sensorTab.add("Left Encoder Raw Graph", 0).withPosition(3,1).withWidget(BuiltInWidgets.kGraph).getEntry();
    private NetworkTableEntry rightRateGraph = sensorTab.add("Right Encoder Rate Graph", 0).withPosition(4,1).withWidget(BuiltInWidgets.kGraph).getEntry();
    private NetworkTableEntry leftRateGraph = sensorTab.add("Left Encoder Rate Graph", 0).withPosition(5,1).withWidget(BuiltInWidgets.kGraph).getEntry();

    private DoubleSolenoid gearShifter = new DoubleSolenoid(0, 1);

    public static Encoder lEncoder =
        new Encoder(Constants.DriveTrain.LEFT_ENCODER_PORT_A, Constants.DriveTrain.LEFT_ENCODER_PORT_B, false, Encoder.EncodingType.k4X);
    public static Encoder rEncoder =
        new Encoder(Constants.DriveTrain.RIGHT_ENCODER_PORT_A, Constants.DriveTrain.RIGHT_ENCODER_PORT_B, false, Encoder.EncodingType.k4X);

    private static AHRS gyro = new AHRS(SPI.Port.kMXP);

    private WPI_VictorSPX flMotor        = new WPI_VictorSPX(Constants.DriveTrain.FL_MOTOR_PORT);
    private WPI_VictorSPX blMotor        = new WPI_VictorSPX(Constants.DriveTrain.BL_MOTOR_PORT);
    private SpeedControllerGroup lMotors = new SpeedControllerGroup(flMotor, blMotor);

    private WPI_VictorSPX frMotor        = new WPI_VictorSPX(Constants.DriveTrain.FR_MOTOR_PORT);
    private WPI_VictorSPX brMotor        = new WPI_VictorSPX(Constants.DriveTrain.BR_MOTOR_PORT);
    private SpeedControllerGroup rMotors = new SpeedControllerGroup(frMotor, brMotor);

    public DifferentialDrive mDrive = new DifferentialDrive(lMotors, rMotors);

    public DriveTrain() {
        mDrive.setSafetyEnabled(true);
        lEncoder.setDistancePerPulse(Constants.DriveTrain.DISTANCE_PER_PULSE);
        rEncoder.setDistancePerPulse(Constants.DriveTrain.DISTANCE_PER_PULSE);
    }

    /**
     * Reset both of encoders
     */
    public void clearEncoder() {
        lEncoder.reset();
        rEncoder.reset();
    }

    /**
     * Print the encoder values, left (L Encoder Distance) and right (R Encoder Distance)
     */

    public double getLeftEncoder(){
        lEncoder.getDistance();
    }

    public double getRightEncoder(){
       rEncoder.getDistance();
    }

    public printEncoderDistance() {
        rightDistance.setDouble(rEncoder.getDistance());
        leftDistance.setDouble(lEncoder.getDistance());
        rightRaw.setDouble(rEncoder.getRaw());
        leftRaw.setDouble(lEncoder.getRaw());
        rightRate.setDouble(rEncoder.getRate());
        leftRate.setDouble(lEncoder.getRate());

        rightDistanceGraph.setDouble(rEncoder.getDistance());
        leftDistanceGraph.setDouble(lEncoder.getDistance());
        rightRawGraph.setDouble(rEncoder.getRaw());
        leftRawGraph.setDouble(lEncoder.getRaw());
        rightRateGraph.setDouble(rEncoder.getRate());
        leftRateGraph.setDouble(lEncoder.getRate());
    }
    /**
     * Reset the gyro to zero
     * Avoid usage at all costs
     */
    public void clearGyro() { gyro.reset(); }

    public void getAngle() {
        gyro.getYaw();
    }

    public double getYaw() { return gyro.getYaw(); }

    public void setBrakeMode() {
        frMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        brMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        blMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        flMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    }

    public void setCoastMode() {
        frMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        brMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        blMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        flMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    }

    public void setHighGear() { gearShifter.set(DoubleSolenoid.Value.kForward); }

    public void setLowGear() { gearShifter.set(DoubleSolenoid.Value.kReverse); }

    protected void initDefaultCommand() { setDefaultCommand(new Drive()); }
}