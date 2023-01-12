package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private WPI_TalonFX clawOpen;
    private WPI_TalonFX clawClose;
    
    public ClawSubsystem() {
        clawOpen = new WPI_TalonFX(8);
        clawClose = new WPI_TalonFX(9);

        clawOpen.setNeutralMode(NeutralMode.Brake);
        clawClose.setNeutralMode(NeutralMode.Brake);
    }

    public void setClimbMotors(double speed) {
        clawOpen.set(ControlMode.PercentOutput, speed);
        clawClose.set(ControlMode.PercentOutput, -speed);
    }

    public double getRightClimbEncoder() {
        return clawClose.getSelectedSensorPosition();
    }

    public void ResetClimbEncoders() {
        clawClose.setSelectedSensorPosition(0);
        clawOpen.setSelectedSensorPosition(0);
    }
}