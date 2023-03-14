package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class SwerveModule {
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    //PIDController drivePID = new PIDController(3.1679, 0, 0);
    ProfiledPIDController drivePID = new ProfiledPIDController(3.1679, 0, 0, new Constraints(Constants.maxModuleVelocity, Constants.maxModuleVelocity));
    PIDController turnPID = new PIDController(4.1807, 0, 0.23405);

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(-0.095829, 2.7601, 0.71108);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0.24959, 0.3754, 0.0068821);

    public SwerveModule(int driveMotorIndex, int turnMotorIndex, boolean driveInverted, boolean turnInverted) {
        driveMotor = new WPI_TalonFX(driveMotorIndex);
        turnMotor = new WPI_TalonFX(turnMotorIndex);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(turnInverted);
        resetModule();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void stopModule() {
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    public void setModule(double driveVolts, double turnVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            new Rotation2d(getCurrentAngle())
        );
    }

    public void resetModule() {
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(0);
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
        return driveMotor.getSelectedSensorVelocity() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * Math.PI * Constants.swerveWheelDiameter;
    }

    private double getDriveDistance() {
        return driveMotor.getSelectedSensorPosition() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * Math.PI * Constants.swerveWheelDiameter;
    }

    private double getCurrentAngle() {
        return turnMotor.getSelectedSensorPosition() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
    }
}
