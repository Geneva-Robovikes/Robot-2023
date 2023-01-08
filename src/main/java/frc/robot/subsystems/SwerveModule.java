package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class SwerveModule {
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX turnMotor) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition() * Constants.swerveDriveGearRatio,
            new Rotation2d(turnMotor.getSelectedSensorPosition() * Constants.swerveTurnGearRatio / 2048 * 2 * Math.PI)
        );
    }
}
