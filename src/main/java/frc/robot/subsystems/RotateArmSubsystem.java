package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotateArmSubsystem extends SubsystemBase {
    private WPI_TalonFX rotateArmMotor;

    public RotateArmSubsystem() {
        rotateArmMotor = new WPI_TalonFX(11);
    }

    public void setArmRotationMotor(double speed) {
        rotateArmMotor.set(speed);
    }
}
