package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;

public class CommandClawGen {


    private static final PneumaticsControlModule pcm = new PneumaticsControlModule();
    private static final int channel4 = 4;
    private static final int channel5 = 5;
    private static final int channel6 = 6;
    private static final Solenoid solenoid4 = pcm.makeSolenoid(channel4);
    private static final Solenoid solenoid5 = pcm.makeSolenoid(channel5);
    private static final Solenoid solenoid6 = pcm.makeSolenoid(channel6);

    public static CommandClaw create(){
        return new CommandClaw(solenoid4, solenoid5, solenoid6);
    }


}
