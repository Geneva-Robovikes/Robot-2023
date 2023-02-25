package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase{
    private WPI_TalonFX pivotMotor;

    public PivotSubsystem () {
        pivotMotor = new WPI_TalonFX(9);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setPivotMotor(double value) {
        System.out.println("heresubsystem");
        pivotMotor.set(ControlMode.PercentOutput, value);
    }
}
