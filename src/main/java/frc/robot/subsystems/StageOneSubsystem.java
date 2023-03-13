package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StageOneSubsystem extends SubsystemBase { 
    private WPI_TalonFX armExtendMotor;
    DigitalInput stageOneLimitSwitchBottom;
    DigitalInput stageOneLimitSwitchTop;

    public StageOneSubsystem () {
        //TODO: fix all of these they're weird. this one is actually incorrect.
        stageOneLimitSwitchBottom = new DigitalInput(4);
        stageOneLimitSwitchTop = new DigitalInput(5);
        armExtendMotor = new WPI_TalonFX(13);
        armExtendMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setarmExtendMotor(double value){
        armExtendMotor.set(ControlMode.PercentOutput, value);
    }

    public boolean getTopState() {
        return stageOneLimitSwitchTop.get();
    }

    public boolean getBottomState() {
        return stageOneLimitSwitchBottom.get();
    }

    public boolean getSwitchState() {
        return (stageOneLimitSwitchBottom.get()||stageOneLimitSwitchTop.get());
    }

    public double getDistance() {
        return armExtendMotor.getSelectedSensorPosition();
    }

    public void resetDistance() {
        armExtendMotor.setSelectedSensorPosition(0);
    }
}