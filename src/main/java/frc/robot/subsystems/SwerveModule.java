package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    /* TODO: Tune to robot values
    ProfiledPIDController drivePID = new ProfiledPIDController(
        8, 
        28, 
        0.1, 
        new TrapezoidProfile.Constraints(6, 6)
    );
    ProfiledPIDController turnPID = new ProfiledPIDController(
        1, 
        10,
        0,
        new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 2)
    );
    */

    //TODO: Tune to robot values
    PIDController drivePID = new PIDController(5.6234, 0, 0);
    PIDController turnPID = new PIDController(4.1692, 0, 0.23252);
    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(-0.14776, 2.9203, 0.42973);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0.26843, 0.36421, 0.010068);

    public SwerveModule(int driveMotorIndex, int turnMotorIndex, boolean driveInverted, boolean turnInverted) {
        driveMotor = new WPI_TalonFX(driveMotorIndex);
        turnMotor = new WPI_TalonFX(turnMotorIndex);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(turnInverted);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
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
        //double turnFeed = turnFeedForward.calculate(state.angle.getRadians());
        
        if(Math.abs(driveOutput + driveFeed) > 0.75) {
            driveMotor.setVoltage(driveOutput + driveFeed);
        } else {
            driveMotor.setVoltage(0);
        }
        
        
        if(Math.abs(turnOutput) > 0.75) {
            turnMotor.setVoltage(turnOutput);
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
        SmartDashboard.putNumber("" + turnMotor.getDeviceID(), -turnMotor.getSelectedSensorPosition() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI);
        return turnMotor.getSelectedSensorPosition() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
    }
}
