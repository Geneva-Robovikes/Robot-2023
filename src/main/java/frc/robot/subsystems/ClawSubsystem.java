package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private WPI_TalonFX clawMotor;
    
    public ClawSubsystem() {
        clawMotor = new WPI_TalonFX(8);

        clawMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setClawState(double speed) {
        clawMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getClawEncoder() {
        return clawMotor.getSelectedSensorPosition();
    }

    public void ResetClawEncoders() {
        clawMotor.setSelectedSensorPosition(0);
    }
}