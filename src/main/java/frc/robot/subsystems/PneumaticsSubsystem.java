package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticHub;

public class PneumaticsSubsystem extends SubsystemBase {
    Compressor phCompressor;
    DoubleSolenoid exampleDoublePH;

    public PneumaticsSubsystem() {

        phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
        exampleDoublePH = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        exampleDoublePH.set(kReverse);

    }

    
    public boolean getPressureSwitch() {
        return phCompressor.getPressureSwitchValue();

    }

    
    public double getCurrent() {
        return phCompressor.getCurrent();
    }

    public void setSolenoid(Value value) {
        exampleDoublePH.set(value);
    }


}
