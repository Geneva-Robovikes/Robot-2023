package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
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
    PIDController drivePID = new PIDController(1, 0, 0);
    ProfiledPIDController turnPID = new ProfiledPIDController(
        1, 
        0,
        0,
        new TrapezoidProfile.Constraints(Constants.moduleMaxAngularVelocity, Constants.moduleMaxAngularAcceleration)
    );

    //TODO: Tune to robot values
    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0, 0);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0, 0);

    public SwerveModule(int driveMotorIndex, int turnMotorIndex, boolean inverted) {
        driveMotor = new WPI_TalonFX(driveMotorIndex);
        turnMotor = new WPI_TalonFX(turnMotorIndex);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveMotor.setInverted(inverted);
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
        return driveMotor.getSelectedSensorVelocity() / Constants.falconEncoderResolution / Constants.swerveDriveGearRatio * 2 * Math.PI * Constants.swerveWheelRadius;
    }

    private double getDriveDistance() {
        return driveMotor.getSelectedSensorPosition() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * 2 * Math.PI * Constants.swerveWheelRadius;
    }

    private double getCurrentAngle() {
        double angle = turnMotor.getSelectedSensorPosition() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
        double multiple = (int) (angle / (2 * Math.PI));
        return angle - (2 * Math.PI * multiple);
    }
}
