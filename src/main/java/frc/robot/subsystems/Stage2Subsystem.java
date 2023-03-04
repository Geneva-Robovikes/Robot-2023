package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stage2Subsystem extends SubsystemBase{
    WPI_TalonFX upperArmMotor;

    public Stage2Subsystem() {
        upperArmMotor = new WPI_TalonFX(10);
    }

    public void setUpperMotor(double speed) {
        upperArmMotor.set(speed);
    }
}
