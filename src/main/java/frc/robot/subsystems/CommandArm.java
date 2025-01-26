package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static edu.wpi.first.units.Units.MetersPerSecond;



public class CommandArm implements Subsystem {

    private final SparkMax motor0;
    private final SimpleMotorFeedforward motor0Feedforward;
    private final SlewRateLimiter slewRateLimiter0;
    private final DCMotorSim motor0Sim;
    private final MutLinearVelocity motor0Setpoint = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity motor0Velocity = MetersPerSecond.mutable(0.0);
    private final SparkMaxSim spark0Sim;
    private final double motor0Ks;

    public CommandArm(
            final SparkMax motor0,
            final SimpleMotorFeedforward motor0Feedforward,
            final SlewRateLimiter slewRateLimiter0,
            final DCMotorSim motor0Sim,
            final DCMotor dcMotor0) {
            this.motor0 = motor0;
            this.motor0Feedforward = motor0Feedforward;
            this.slewRateLimiter0 = slewRateLimiter0;
            this.motor0Sim = motor0Sim;
            this.spark0Sim = new SparkMaxSim(motor0, dcMotor0);
            motor0Ks = motor0Feedforward.getKs();
    }

    private void updateTelemetry() {
        motor0Velocity.mut_replace(motor0.getEncoder().getVelocity(), MetersPerSecond);
    }

    // private methods
    private void applySetpoint0() {
        double velocitySetpoint = slewRateLimiter0.calculate(motor0Setpoint.baseUnitMagnitude());
        double currentVelocity = motor0.getEncoder().getVelocity();
        double arbFeedforward = motor0Feedforward.calculateWithVelocities(currentVelocity, velocitySetpoint);
        arbFeedforward = MathUtil.clamp(arbFeedforward, -12.0, 12.0);
        motor0.getClosedLoopController().setReference(velocitySetpoint, kVelocity, kSlot1, arbFeedforward, kVoltage);
    }


    private void stopMotor0() {
        motor0Setpoint.mut_replace(0.0, MetersPerSecond);
        motor0.stopMotor();
    }


    private void stop() {
        stopMotor0();
    }

    private void updateMotorSim0() {
        double voltage = motor0.getAppliedOutput() * motor0.getBusVoltage();
        if (voltage >= 0 && voltage <= motor0Ks) {
            voltage = 0;
        } else if (voltage < 0 && voltage >= -motor0Ks) {
            voltage = 0;
        }
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
    public Trigger motor0AtSetpoint(LinearVelocity velocitySetpoint, LinearVelocity velocitySetpointTolerance) {
        return new Trigger(() -> motor0Velocity.isNear(velocitySetpoint, velocitySetpointTolerance));
    }


    public Trigger motorsAtSetpoints(LinearVelocity motor0Setpoint, LinearVelocity motor0SetpointTolerance) {
            return motor0AtSetpoint(motor0Setpoint, motor0SetpointTolerance);
    }

    public Command createStop() {
        return runOnce(this::stop).withName("Stop");
    }

    public Command createApplySetpoint(String name, LinearVelocity motor0Setpoint) {
        return runOnce(() -> {
            this.motor0Setpoint.mut_replace(motor0Setpoint);
        }).withName(name);
    }


}


