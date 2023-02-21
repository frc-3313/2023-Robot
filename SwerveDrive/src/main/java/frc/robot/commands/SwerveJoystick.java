package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends CommandBase{
    private SwerveSubsystem swerveSubsystem;

    public double xSpeed, ySpeed, turningSpeed;
    private boolean fieldOriented;
    private SlewRateLimiter xlimiter, ylimiter;
    private SlewRateLimiter turnLimiter;
    private Supplier<Double> xSpeedF, ySpeedF, turningSpeedF;
    public SwerveJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeedF, Supplier<Double> ySpeedF, Supplier<Double> turningSpeedF, boolean fieldOriented) {
        //this.fieldOriented = false;
        //this.limiter = new SlewRateLimiter(0.5, -0.5, 0);
        //this.turnLimiter = new SlewRateLimiter(0.4, -0.4, 0);
        //this.controller = new PS4Controller(0);
        //
        //this.turningSpeed = controller.getRightX();
        //this.xSpeed = controller.getLeftX();
        //this.ySpeed = controller.getLeftY();
        this.fieldOriented = fieldOriented;
        this.xlimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.ylimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedF = xSpeedF;
        this.ySpeedF = ySpeedF;
        this.turningSpeedF = turningSpeedF;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        xSpeed = xSpeedF.get() / 2;
        ySpeed = ySpeedF.get() / 2;
        turningSpeed = (turningSpeedF.get());

        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xlimiter.calculate(xSpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        ySpeed = ylimiter.calculate(ySpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        //get integer input of controller and using that as a percent of 2 pi to figure out how much to turn 
        turningSpeed = (turnLimiter.calculate(turningSpeed) * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond);

        ChassisSpeeds chassisSpeeds;
        if(fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            SmartDashboard.putNumber("Drive xSpeed", xSpeed);
            SmartDashboard.putNumber("Drive ySpeed", ySpeed);
            SmartDashboard.putNumber("Drive TurningSpeed", turningSpeed);
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }   

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
