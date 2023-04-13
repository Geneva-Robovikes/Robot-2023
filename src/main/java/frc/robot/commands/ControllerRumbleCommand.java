package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleCommand extends CommandBase  {
    private final CommandXboxController controller;
    private final double intensity;
    private final double rumbleTime;
    private Timer timer = new Timer();

    /**
     * Command that rumbles the controller at the intensiy for the specified time.
     * @param controller the controller to rumble
     * @param intensity the intesity of the rumble. Must be between 0.0 and 1.0
     * @param rumbleTime the time to rumble the controller for
     */
    public ControllerRumbleCommand(CommandXboxController controller, double intensity, double rumbleTime) {
        this.controller = controller;
        this.rumbleTime = rumbleTime;
        this.intensity = intensity;
    }

    @Override
    public void initialize() {
        timer.reset();
        controller.getHID().setRumble(RumbleType.kBothRumble, intensity);
        timer.start();
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > rumbleTime;
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
}
