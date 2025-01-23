package frc.robot.subsystems;


import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class CommandIntake implements Subsystem {

    private final SparkMax motor0;
    private final SimpleMotorFeedforward motor0Feedforward;
    private final DCMotorSim motor0Sim;
    private final MutAngularVelocity previousMotor0Setpoint = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity nextSetpoint = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity motor0Setpoint = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity motor0Velocity = RadiansPerSecond.mutable(0.0);
    private final SparkMaxSim spark0Sim;
    private final double motor0Ks;
    private final SimpleMotorFeedforward simFeedForward;

    public CommandIntake(final SparkMax motor0,
                         final SimpleMotorFeedforward motor0Feedforward,
                         final DCMotorSim motor0Sim,
                         final DCMotor dcMotor0) {
        this.motor0 = motor0;
        this.motor0Feedforward = motor0Feedforward;
        this.motor0Sim = motor0Sim;
        this.spark0Sim = new SparkMaxSim(motor0, dcMotor0);
        motor0Ks = motor0Feedforward.getKs();
        this.simFeedForward = new SimpleMotorFeedforward(0, motor0Feedforward.getKv(), motor0Feedforward.getKa(), motor0Feedforward.getDt());
    }

    private void updateTelemetry() {
        motor0Velocity.mut_replace(motor0.getEncoder().getVelocity(), RadiansPerSecond);
        SmartDashboard.putNumber("Velocity", motor0Velocity.baseUnitMagnitude());
    }

    // private methods
    private void applySetpoint0() {
        double nextVelocitySetpoint = motor0Setpoint.baseUnitMagnitude();
        double currentVelocitySetpoint = previousMotor0Setpoint.baseUnitMagnitude();
        previousMotor0Setpoint.mut_replace(nextVelocitySetpoint, RadiansPerSecond);
        nextSetpoint.mut_replace(nextVelocitySetpoint, RadiansPerSecond);
        double arbFeedforward = motor0Feedforward.calculateWithVelocities(currentVelocitySetpoint, nextVelocitySetpoint);
        previousMotor0Setpoint.mut_replace(motor0Setpoint);
        motor0.getClosedLoopController().setReference(nextVelocitySetpoint, kMAXMotionVelocityControl, kSlot1, arbFeedforward, kVoltage);
    }

    private void stopMotor0() {
        motor0Setpoint.mut_replace(0.0, RadiansPerSecond);
    }

    private void stop() {
        stopMotor0();
    }

    private void updateMotorSim0() {
        double voltage = motor0.getAppliedOutput() * (12.0);
        motor0Sim.setInputVoltage(voltage);
        motor0Sim.update(0.01);
        spark0Sim.iterate(motor0Sim.getAngularVelocityRadPerSec(), 12.0, 0.01);

    }

    @Override
    public void periodic() {
        updateTelemetry();
        applySetpoint0();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSim0();
    }

    // triggers factories
    public Trigger motor0AtSetpoint(AngularVelocity velocitySetpoint, AngularVelocity velocitySetpointTolerance) {
        return new Trigger(() -> motor0Velocity.isNear(velocitySetpoint, velocitySetpointTolerance));
    }

    public Trigger motorsAtSetpoints(
            AngularVelocity motor0Setpoint,
            AngularVelocity motor0SetpointTolerance) {
        return motor0AtSetpoint(motor0Setpoint, motor0SetpointTolerance);
    }

    public Command createStop() {
        return runOnce(this::stop).withName("Stop");
    }

    public Command createApplySetpoint(String name, AngularVelocity motor0Setpoint) {
        return runOnce(() -> {
            this.motor0Setpoint.mut_replace(motor0Setpoint);
        }).withName(name);
    }


}

