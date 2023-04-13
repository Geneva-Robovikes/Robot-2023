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

    //PID controllers allow for accurate position/velocity tracking
    //Profiled PID controller is an extension of PID controllers that allows for velocity and acceleration constraints
    //These are feedback controllers, so they correct for error
    ProfiledPIDController drivePID = new ProfiledPIDController(3.1679, 0, 0, new Constraints(Constants.maxModuleVelocity, Constants.maxModuleVelocity));
    PIDController turnPID = new PIDController(4.1807, 0, 0.23405);

    //Feedforward controllers anticipate motion
    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(-0.095829, 2.7601, 0.71108);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0.24959, 0.3754, 0.0068821);

    /**
     * Creates a swerve module!
     * @param driveMotorIndex The index of the driving motor
     * @param turnMotorIndex The index of the turning motor
     * @param driveInverted True if the drive motor should be inverted
     * @param turnInverted True if the turn motor should be inverted
     */
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

    /**
     * Sets the drive and turn voltages to zero
     */
    public void stopModule() {
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    /**
     * Sets the drive and turn motors to the specified speeds
     * @param driveVolts Voltage to set to the drive motor. Must be from -12.0 to 12.0
     * @param turnVolts Voltage to set to the turn motor. Must be from -12.0 to 12.0
     */
    public void setModule(double driveVolts, double turnVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    /**
     * Returns the current position of the swerve module.
     * @return A new SwerveModulePosition with the drive motor's distance and the turn motor's angle
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            new Rotation2d(getCurrentAngle())
        );
    }

    /**
     * Sets the drive and turn motor's position back to 0.
     */
    public void resetModule() {
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(0);
    }

    /**
     * Drives the swerve module from the desired state.
     * @param desiredState The state to attempt to reach
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // SwerveModuleState.optimize() fixes wheel rotation making it more efficient.
        // In simple terms, turning 180 degrees and driving forward is the same as turning 90 degrees and drving backwards.
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));

        // The calculate method gets the result from the controllers.
        // For feedback, they take in the current position and the target position.
        // For feedforward, they take in the target position.
        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double driveFeed = driveFeedForward.calculate(state.speedMetersPerSecond);
        double turnOutput = turnPID.calculate(getCurrentAngle(), state.angle.getRadians());
        
        // The PID controllers will return very small decimals which will leave the motors at a constant stall.
        // This can damage the motors, so this deadzone is a minimum voltage to be returned to drive the module.
        if(Math.abs(driveOutput + driveFeed) > 0.75) {
            //To implement feedback and feedforward controllers add their outputs!
            driveMotor.setVoltage(driveOutput + driveFeed);
        } else {
            driveMotor.setVoltage(0);
        }
        
        // Same idea as above!
        if(Math.abs(turnOutput) > 0.75) {
            turnMotor.setVoltage(turnOutput);
        } else {
            turnMotor.setVoltage(0);
        }
        
    }

    /**
     * Returns the current wheel velocity of the module.
     * @return The velocity in meters per second.
     */
    private double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * Math.PI * Constants.swerveWheelDiameter;
    }

    /**
     * Returns the current wheel position of the module.
     * @return The position of the wheel in meters.
     */
    private double getDriveDistance() {
        return driveMotor.getSelectedSensorPosition() / Constants.swerveDriveGearRatio / Constants.falconEncoderResolution * Math.PI * Constants.swerveWheelDiameter;
    }

    /**
     * Returns the current wheel angle of the module.
     * @return The angle of the wheel in radians.
     */
    private double getCurrentAngle() {
        return turnMotor.getSelectedSensorPosition() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
    }
}
