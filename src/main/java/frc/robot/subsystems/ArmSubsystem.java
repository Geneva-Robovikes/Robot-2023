package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private final WPI_TalonFX upperExtensionMotor;
    private final WPI_TalonFX lowerExtensionMotor;
    private final WPI_TalonFX armPivotMotor;
    
    private final DigitalInput stageOneLimitSwitchTop;
    private final DigitalInput stageOneLimitSwitchBottom;
    private final DigitalInput stageTwoLimitSwitchTop;
    private final DigitalInput stageTwoLimitSwitchBottom;
    private final DigitalInput clawArmLimitSwitchUp;
    private final DigitalInput clawArmLimitSwitchDown;


    public ArmSubsystem() {
        upperExtensionMotor = new WPI_TalonFX(10);
        armPivotMotor = new WPI_TalonFX(12);
        lowerExtensionMotor = new WPI_TalonFX(13);
        
        clawArmLimitSwitchUp = new DigitalInput(0);
        clawArmLimitSwitchDown = new DigitalInput(1);
        stageOneLimitSwitchTop = new DigitalInput(5);
        stageOneLimitSwitchBottom = new DigitalInput(4);
        stageTwoLimitSwitchTop = new DigitalInput(6);
        stageTwoLimitSwitchBottom = new DigitalInput(7);

        upperExtensionMotor.setInverted(true);

        upperExtensionMotor.setNeutralMode(NeutralMode.Brake);
        lowerExtensionMotor.setNeutralMode(NeutralMode.Brake);
        armPivotMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void setUpperExensionMotor(double speed) {
        upperExtensionMotor.set(speed);
    }

    public boolean getUpperExensionTopState() {
        return stageTwoLimitSwitchTop.get();
    }

    public boolean getUpperExensionBottomState() {
        return stageTwoLimitSwitchBottom.get();
    }

    public double getUpperExensionDistance() {
        return upperExtensionMotor.getSelectedSensorPosition();
    }

    public void resetUpperExensionDistance() {
        upperExtensionMotor.setSelectedSensorPosition(0);
    }

    public void setLowerExtensionMotor(double speed){
        lowerExtensionMotor.set(speed);
    }

    public boolean getLowerExtensionTopState() {
        return stageOneLimitSwitchTop.get();
    }

    public boolean getLowerExtensionBottomState() {
        return stageOneLimitSwitchBottom.get();
    }

    public double getLowerExtensionDistance() {
        return lowerExtensionMotor.getSelectedSensorPosition();
    }

    public void resetLowerExtensionDistance() {
        lowerExtensionMotor.setSelectedSensorPosition(0);
    }

    public void setArmPivotMotor(double speed) {
        armPivotMotor.set(speed);
    }

    public void resetArmPivotEncoder() {
        armPivotMotor.setSelectedSensorPosition(0);
    }

    public double getArmPivotPosition() {
        return armPivotMotor.getSelectedSensorPosition();
    }

    public boolean getArmPivotUpSwitch() {
        return !clawArmLimitSwitchUp.get();
    }

    public boolean getArmPivotDownSwitch() {
        return clawArmLimitSwitchDown.get();
    }

    public double getArmPivotAngle() {
        return armPivotMotor.getSelectedSensorPosition() / Constants.falconEncoderResolution / Constants.clawAngleGearRatio * (2 * Math.PI);
    }
}
