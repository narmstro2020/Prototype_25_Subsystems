package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandClaw implements Subsystem {

    private final BooleanPublisher state0Publisher;
    private final BooleanPublisher state1Publisher;
    private final BooleanPublisher state2Publisher;

    private final Solenoid solenoid0;
    private final Solenoid solenoid1;
    private final Solenoid solenoid2;

    public CommandClaw(Solenoid solenoid0, Solenoid solenoid1, Solenoid solenoid2) {
        this.solenoid0 = solenoid0;
        this.solenoid1 = solenoid1;
        this.solenoid2 = solenoid2;

        this.state0Publisher = NetworkTableInstance
                .getDefault()
                .getTable("Claw")
                .getBooleanTopic("State0")
                .publish();
        this.state1Publisher = NetworkTableInstance
                .getDefault()
                .getTable("Claw")
                .getBooleanTopic("State1")
                .publish();
        this.state2Publisher = NetworkTableInstance
                .getDefault()
                .getTable("Claw")
                .getBooleanTopic("State2")
                .publish();
    }

    private void updateTelemetry(){
        state0Publisher.set(solenoid0.get());
        state1Publisher.set(solenoid1.get());
        state2Publisher.set(solenoid2.get());
    }

    private void extend0(){
        solenoid0.set(true);
    }

    private void contract0(){
        solenoid0.set(false);
    }

    private void extend1(){
        solenoid1.set(true);
    }

    private void contract1(){
        solenoid1.set(false);
    }

    private void extend2(){
        solenoid2.set(true);
    }

    private void contract2(){
        solenoid2.set(false);
    }


    @Override
    public void periodic() {
        updateTelemetry();
    }

    public Trigger is0Extended(){
        return new Trigger(solenoid0::get);
    }

    public Trigger is0Contracted(){
        return new Trigger(() -> !solenoid0.get());
    }

    public Trigger is1Extended(){
        return new Trigger(solenoid1::get);
    }

    public Trigger is1Contracted(){
        return new Trigger(() -> !solenoid1.get());
    }

    public Trigger is2Extended(){
        return new Trigger(solenoid2::get);
    }

    public Trigger is2Contracted(){
        return new Trigger(() -> !solenoid2.get());
    }

    public Command create0Extend(){
        return runOnce(this::extend0).withName("Extend0");
    }

    public Command create0Contract(){
        return runOnce(this::contract0).withName("Contract0");
    }

    public Command create1Extend(){
        return runOnce(this::extend1).withName("Extend1");
    }

    public Command create1Contract(){
        return runOnce(this::contract1).withName("Contract1");
    }

    public Command creat2Extend(){
        return runOnce(this::extend2).withName("Extend2");
    }

    public Command create2Contract(){
        return runOnce(this::contract2).withName("Contract2");
    }

    public Command createToggle0(){
        return runOnce(solenoid0::toggle).withName("Toggle0");
    }

    public Command createToggle1(){
        return runOnce(solenoid1::toggle).withName("Toggle1");
    }

    public Command createToggle2(){
        return runOnce(solenoid2::toggle).withName("Toggle2");
    }
}
