package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StageTwoSubsystem extends SubsystemBase{
    WPI_TalonFX upperArmMotor;
    DigitalInput stageTwoLimitSwitchBottom;
    DigitalInput stageTwoLimitSwitchTop;

    public StageTwoSubsystem() {
        stageTwoLimitSwitchBottom = new DigitalInput(7);
        stageTwoLimitSwitchTop = new DigitalInput(6);
        upperArmMotor = new WPI_TalonFX(10);
        upperArmMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setUpperMotor(double speed) {
        upperArmMotor.set(speed);
    }

    public boolean getTopState() {
        return stageTwoLimitSwitchTop.get();
    }

    public boolean getBottomState() {
        return stageTwoLimitSwitchBottom.get();
    }

    public boolean getSwitchState() {
        return (stageTwoLimitSwitchBottom.get()||stageTwoLimitSwitchTop.get());
    }

    public double getDistance() {
        return upperArmMotor.getSelectedSensorPosition();
    }

    public void resetDistance() {
        upperArmMotor.setSelectedSensorPosition(0);
    }
}
