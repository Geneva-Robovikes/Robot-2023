package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {
    private final WPI_TalonFX pivotMotor;
    private final CANSparkMax clawMotor;
    private final DigitalInput pivotLimitSwitchTop;
    private final DigitalInput pivotLimitSwitchBottom;

    /**
     * Creates the Claw Subsystem. This controls claw rotation and intaking/outtaking.
     */
    public ClawSubsystem(){
        pivotMotor = new WPI_TalonFX(9);
        clawMotor = new CANSparkMax(8, MotorType.kBrushless);
        
        pivotLimitSwitchTop = new DigitalInput(2);
        pivotLimitSwitchBottom = new DigitalInput(3);

        clawMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the intake motor to the specified speed.
     * @param speed The motor speed. Must be between -1.0 and 1.0
     */
    public void setClawMotor(double speed){
        clawMotor.set(speed);
    }

    /**
     * Returns the applied current to the intake motor.
     * @return the applied current in amps.
     */
    public double getClawMotorCurrent() {
        SmartDashboard.putNumber("Neo Current", clawMotor.getOutputCurrent());
        return clawMotor.getOutputCurrent();
    }

    /**
     * Sets the pivot motor to the specified speed.
     * @param speed The speed to set the motor. Must be between -1.0 and 1.0
     */
    public void setPivotMotor(double speed){
        pivotMotor.set(speed);
    }

    /**
     * Returns the state of the pivot's top limit switch.
     * @return The top switch state.
     */
    public boolean getPivotTopState() {
        return !pivotLimitSwitchTop.get();
    }

    /**
     * Returns the state of the pivot's bottom limit switch.
     * @return The top switch state.
     */
    public boolean getPivotBottomState() {
        return !pivotLimitSwitchBottom.get();
    }

    /**
     * Returns the distance the pivot motor has travelled.
     * @return The distance the motor has travelled in encoder units.
     */
    public double getPivotDistance() {
        return pivotMotor.getSelectedSensorPosition();
    }
    
    /**
     * Reset the position of the pivot motor encoder.
     */
    public void resetPivotEncoder() {
        pivotMotor.setSelectedSensorPosition(0);
    }

    /**
     * Returns the sngle the claw has travelled.
     * @return The angle of the claw in radians.
     */
    public double getClawAngle() {
        return pivotMotor.getSelectedSensorPosition() / Constants.falconEncoderResolution / Constants.clawAngleGearRatio * (2 * Math.PI);
    }
}
