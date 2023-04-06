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

    /**
     * Creates the Arm Subsystem. This controlls both stages of the extension and the rotation of the arm.
     */
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

        lowerExtensionMotor.setInverted(true);

        upperExtensionMotor.setNeutralMode(NeutralMode.Brake);
        lowerExtensionMotor.setNeutralMode(NeutralMode.Brake);
        armPivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Set the speed of the upper extension motor.
     * @param speed The speed to set the motor. Must be between -1.0 and 1.0
     */
    public void setUpperExensionMotor(double speed) {
        upperExtensionMotor.set(speed);
    }

    /**
     * Get the state of the upper extension's top switch.
     * @return The state of the limit switch.
     */
    public boolean getUpperExensionTopState() {
        return !stageTwoLimitSwitchTop.get();
    }

    /**
     * Get the state of the upper extension's bottom switch.
     * @return The state of the limit switch.
     */
    public boolean getUpperExensionBottomState() {
        return !stageTwoLimitSwitchBottom.get();
    }

    /**
     * Get the distance the upper extension motor has travelled.
     * @return the distance the motor traveled in encoder units.
     */
    public double getUpperExensionDistance() {
        return upperExtensionMotor.getSelectedSensorPosition();
    }

    /**
     * Reset the position of the upper extension motor.
     */
    public void resetUpperExensionDistance() {
        upperExtensionMotor.setSelectedSensorPosition(0);
    }

    /**
     * Set the speed of the lower extension motor.
     * @param speed The speed to set the motor. Must be between -1.0 and 1.0
     */
    public void setLowerExtensionMotor(double speed){
        lowerExtensionMotor.set(speed);
    }

    /**
     * Get the state of the lower extension's top switch.
     * @return The state of the limit switch.
     */
    public boolean getLowerExtensionTopState() {
        return !stageOneLimitSwitchTop.get();
    }

    /**
     * Get the state of the lower extension's bottom switch.
     * @return The state of the limit switch.
     */
    public boolean getLowerExtensionBottomState() {
        return !stageOneLimitSwitchBottom.get();
    }

    /**
     * Get the distance the lower extension motor has travelled.
     * @return the distance the motor traveled in encoder units.
     */
    public double getLowerExtensionDistance() {
        return lowerExtensionMotor.getSelectedSensorPosition();
    }

    /**
     * Reset the position of the upper lower motor.
     */
    public void resetLowerExtensionDistance() {
        lowerExtensionMotor.setSelectedSensorPosition(0);
    }

    /**
     * Sets the speed of the Arm Pivot motor.
     * @param speed The speed to set the motor. Must be between -1.0 and 1.0
     */
    public void setArmPivotMotor(double speed) {
        armPivotMotor.set(speed);
    }

    /**
     * Reset the position of the Arm Pivot motor.
     */
    public void resetArmPivotEncoder() {
        armPivotMotor.setSelectedSensorPosition(0);
    }

    /**
     * Get the distance the lower extension motor has travelled.
     * @return the distance the motor traveled in encoder units.
     */
    public double getArmPivotPosition() {
        return armPivotMotor.getSelectedSensorPosition();
    }

    /**
     * Get the state of the upper extension's top switch.
     * @return The state of the limit switch.
     */
    public boolean getArmPivotTopState() {
        return !clawArmLimitSwitchUp.get();
    }

    /**
     * Get the state of the upper extension's bottom switch.
     * @return The state of the limit switch.
     */
    public boolean getArmPivotBottomState() {
        return !clawArmLimitSwitchDown.get();
    }

    /**
     * Returns the angle the arm has rotated.
     * @return The angle the arm has rotated in radians.
     */
    public double getArmPivotAngle() {
        return armPivotMotor.getSelectedSensorPosition() / Constants.falconEncoderResolution / Constants.clawAngleGearRatio * (2 * Math.PI);
    }
}
