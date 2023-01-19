package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private WPI_TalonFX elevatorMotor;
    private PIDController pid = new PIDController(0, 0, 0);

    
    public ElevatorSubsystem() {
        elevatorMotor = new WPI_TalonFX(8);


        elevatorMotor.setNeutralMode(NeutralMode.Brake);

    }

    public double degreeToEncoder(int degree) {
        final double encoderDegree = 5.69;
        double encoder = degree / encoderDegree;
        
        return encoder;
    }

    public void setelevatorMotor(double speed, int pos) {
    
        elevatorMotor.set(ControlMode.PercentOutput, speed);
        //should work
        elevatorMotor.set(pid.calculate(elevatorMotor.getSelectedSensorPosition(), degreeToEncoder(pos)));
          
    }

    public double getelevatorEncoder() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void ResetelevatorEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
 
    }
}