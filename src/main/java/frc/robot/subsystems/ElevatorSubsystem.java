package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private WPI_TalonFX elevatorMotor;

    
    public ElevatorSubsystem() {
        elevatorMotor = new WPI_TalonFX(8);


        elevatorMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void setelevatorMotor(double speed) {
        elevatorMotor.set(ControlMode.PercentOutput, speed);

    }

    public double getelevatorEncoder() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void ResetelevatorEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
 
    }
}