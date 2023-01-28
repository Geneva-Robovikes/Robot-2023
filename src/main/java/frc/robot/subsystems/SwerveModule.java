package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;

    // TODO: Tune to robot values
    PIDController drivePID = new PIDController(1, 0, 0);
    ProfiledPIDController turnPID = new ProfiledPIDController(
        1, 
        0,
        0,
        new TrapezoidProfile.Constraints(Constants.moduleMaxAngularVelocity, Constants.moduleMaxAngularAcceleration)
    );

    //TODO: Tune to robot values
    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1, 3);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(1, 0.5);

    public SwerveModule(int driveMotorIndex, int turnMotorIndex) {
        driveMotor = new CANSparkMax(driveMotorIndex, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorIndex, MotorType.kBrushless);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
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

        driveMotor.setVoltage(driveOutput + driveFeed);
        turnMotor.setVoltage(turnOutput + turnFeed);
    }

    private double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity() * Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * 2 * Math.PI * Constants.swerveWheelRadius;
    }

    private double getDriveDistance() {
        return driveMotor.getEncoder().getPosition() * Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * 2 * Math.PI * Constants.swerveWheelRadius;
    }

    private double getCurrentAngle() {
        double angle = turnMotor.getEncoder().getPosition() * Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
        double multiple = (int) (angle / (2 * Math.PI));
        return angle - (2 * Math.PI * multiple);
    }
}
