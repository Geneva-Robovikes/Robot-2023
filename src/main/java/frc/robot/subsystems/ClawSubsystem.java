package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private WPI_TalonFX clawMotor;

    public ClawSubsystem () {
        clawMotor = new WPI_TalonFX(10);
    }

    public void setClawMotor(double value) {
        clawMotor.set(ControlMode.PercentOutput, value);
    }
}
