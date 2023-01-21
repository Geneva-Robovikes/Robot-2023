package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private WPI_TalonFX elevatorMotor;
    private WPI_TalonFX angleMotor;
    private PIDController pid = new PIDController(0, 0, 0);

    //TODO: make motor indexes agruments of the constructor
    public ElevatorSubsystem(int motorIndex, int angleIndex) {
        elevatorMotor = new WPI_TalonFX(motorIndex);
        angleMotor = new WPI_TalonFX(angleIndex);
        //TODO: Add motor to change the arm's angle
        angleMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

    }

    //TODO: Make a mathod to convert from encoder units to elevator length in meters
    public double degreeToEncoder(int degree) {
        final double encoderDegree = 5.69;
        double encoder = degree / encoderDegree;
        
        return encoder;
    }

    public double encoderToMeter(int encoder) {
        final double encoderMeter = 0;
        double meter = encoder / encoderMeter;

        return meter;
    }
    //TODO: make method to set the elevator's length in meters
    public void setelevatorMotor(double speed, int pos) {
    
        elevatorMotor.set(ControlMode.PercentOutput, speed);
        //should work
        //TODO: pass in the current langth in meters and the goal length
        elevatorMotor.set(pid.calculate(elevatorMotor.getSelectedSensorPosition(), degreeToEncoder(pos)));
          
    }

    public double getelevatorEncoder() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void ResetelevatorEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
 
    }

    //TODO: Make methods to get angle motor encoder, convert to degrees, and method (using PID control) to set the angle of the arm
    public double getAngleMotorEncoder(){
        return angleMotor.getSelectedSensorPosition();
    }

    public double encoderToDegrees(int encoder){
        double degrees = 0;
        double encoderDegree = 5.69;

        degrees = encoder / encoderDegree;

        return degrees;
    }

    public void setArmAngle(int pos){
        angleMotor.set(pid.calculate(angleMotor.getSelectedSensorPosition(), degreeToEncoder(pos)));
    }

}