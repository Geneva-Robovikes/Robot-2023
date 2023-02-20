package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private WPI_TalonFX armMotor;

    public ArmSubsystem() {
        armMotor = new WPI_TalonFX(9);
    }

    public void setArmMotor (double value) {
        armMotor.set(ControlMode.PercentOutput, value);
    }
}
