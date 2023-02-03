package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    // TODO: Tune to robot values
    ProfiledPIDController drivePID = new ProfiledPIDController(
        8, 
        28, 
        0.2, 
        new TrapezoidProfile.Constraints(6, 6)
    );
    ProfiledPIDController turnPID = new ProfiledPIDController(
        8, 
        5,
        0,
        new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI)
    );

    //TODO: Tune to robot values
    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.5, 0.5);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0.5, 0.5);

    public SwerveModule(int driveMotorIndex, int turnMotorIndex, boolean driveInverted, boolean turnInverted) {
        driveMotor = new WPI_TalonFX(driveMotorIndex);
        turnMotor = new WPI_TalonFX(turnMotorIndex);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(turnInverted);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            new Rotation2d(getCurrentAngle())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));
        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double driveFeed = driveFeedForward.calculate(state.speedMetersPerSecond);
        double turnOutput = turnPID.calculate(getCurrentAngle(), state.angle.getRadians());
        double turnFeed = turnFeedForward.calculate(turnPID.getSetpoint().velocity);
        
        if(Math.abs(driveOutput + driveFeed) > 1) {
            driveMotor.setVoltage(driveOutput + driveFeed);
        } else {
            driveMotor.setVoltage(0);
        }
        
        if(Math.abs(turnOutput + turnFeed) > 1) {
            turnMotor.setVoltage(turnOutput + turnFeed);
        } else {
            turnMotor.setVoltage(0);
        }
    }

    private double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * 2 * Math.PI * Constants.swerveWheelRadius;
    }

    private double getDriveDistance() {
        return driveMotor.getSelectedSensorPosition() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * 2 * Math.PI * Constants.swerveWheelRadius;
    }

    private double getCurrentAngle() {
        return turnMotor.getSelectedSensorPosition() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
    }
}
