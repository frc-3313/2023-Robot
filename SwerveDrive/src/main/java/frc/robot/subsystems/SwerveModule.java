package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    private final CANCoder cancoder;
    //private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        //this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.cancoder = new CANCoder(absoluteEncoderId);
        CANCoderConfiguration config = new CANCoderConfiguration();
 // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0 / (Math.pow(2, 13) - 1);
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        cancoder.configAllSettings(config);
        
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(Math.toRadians(5));
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return cancoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return cancoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        //angle *= 2.0 * Math.PI;
        //angle -= absoluteEncoderOffsetRad;
        //return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        return (cancoder.getAbsolutePosition() - absoluteEncoderOffsetRad);
    }
/*May switch to getState */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(cancoder.getPosition())
            );
    }
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        cancoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + cancoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
