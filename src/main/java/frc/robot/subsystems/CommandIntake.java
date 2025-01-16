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

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class CommandIntake implements Subsystem {

    private final SparkMax motor0;
    private final SparkMax motor1;
    private final SimpleMotorFeedforward motor0Feedforward;
    private final SimpleMotorFeedforward motor1Feedforward;
    private final SlewRateLimiter slewRateLimiter0;
    private final SlewRateLimiter slewRateLimiter1;
    private final DCMotorSim motor0Sim;
    private final DCMotorSim motor1Sim;
    private final MutLinearVelocity motor0Setpoint = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity motor1Setpoint = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity motor0Velocity = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity motor1Velocity = MetersPerSecond.mutable(0.0);
    private final SparkMaxSim spark0Sim;
    private final SparkMaxSim spark1Sim;
    private final double motor0Ks;
    private final double motor1Ks;

    public CommandIntake(final SparkMax motor0,
                         final SparkMax motor1,
                         final SimpleMotorFeedforward motor0Feedforward,
                         final SimpleMotorFeedforward motor1Feedforward,
                         final SlewRateLimiter slewRateLimiter0,
                         final SlewRateLimiter slewRateLimiter1,
                         final DCMotorSim motor0Sim,
                         final DCMotorSim motor1Sim,
                         final DCMotor dcMotor0,
                         final DCMotor dcMotor1) {
        this.motor0 = motor0;
        this.motor1 = motor1;
        this.motor0Feedforward = motor0Feedforward;
        this.motor1Feedforward = motor1Feedforward;
        this.slewRateLimiter0 = slewRateLimiter0;
        this.slewRateLimiter1 = slewRateLimiter1;
        this.motor0Sim = motor0Sim;
        this.motor1Sim = motor1Sim;
        this.spark0Sim = new SparkMaxSim(motor0, dcMotor0);
        this.spark1Sim = new SparkMaxSim(motor1, dcMotor1);
        motor0Ks = motor0Feedforward.getKs();
        motor1Ks = motor1Feedforward.getKs();
    }

    private void updateTelemetry() {
        motor0Velocity.mut_replace(motor0.getEncoder().getVelocity(), MetersPerSecond);
        motor1Velocity.mut_replace(motor1.getEncoder().getVelocity(), MetersPerSecond);
    }

    // private methods
    private void applySetpoint0() {
        double velocitySetpoint = slewRateLimiter0.calculate(motor0Setpoint.baseUnitMagnitude());
        double currentVelocity = motor0.getEncoder().getVelocity();
        double arbFeedforward = motor0Feedforward.calculateWithVelocities(currentVelocity, velocitySetpoint);
        arbFeedforward = MathUtil.clamp(arbFeedforward, -12.0, 12.0);
        motor0.getClosedLoopController().setReference(velocitySetpoint, kVelocity, kSlot1, arbFeedforward, kVoltage);
    }

    private void applySetpoint1() {
        double velocitySetpoint = slewRateLimiter1.calculate(motor1Setpoint.baseUnitMagnitude());
        double currentVelocity = motor1.getEncoder().getVelocity();
        double arbFeedforward = motor1Feedforward.calculateWithVelocities(currentVelocity, velocitySetpoint);
        arbFeedforward = MathUtil.clamp(arbFeedforward, -12.0, 12.0);
        motor1.getClosedLoopController().setReference(velocitySetpoint, kVelocity, kSlot1, arbFeedforward, kVoltage);
    }

    private void stopMotor0() {
        motor0Setpoint.mut_replace(0.0, MetersPerSecond);
        motor0.stopMotor();
    }

    private void stopMotor1() {
        motor1Setpoint.mut_replace(0.0, MetersPerSecond);
        motor1.stopMotor();
    }

    private void stop() {
        stopMotor0();
        stopMotor1();
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

    private void updateMotorSim1() {
        double voltage = motor1.getAppliedOutput() * motor1.getBusVoltage();
        if (voltage >= 0 && voltage <= motor1Ks) {
            voltage = 0;
        } else if (voltage < 0 && voltage >= -motor1Ks) {
            voltage = 0;
        }
        motor1Sim.setInputVoltage(voltage);
        motor1Sim.update(0.01);
        spark1Sim.iterate(motor1Sim.getAngularVelocityRadPerSec(), 12.0, 0.01);
    }

    @Override
    public void periodic() {
        updateTelemetry();
        applySetpoint0();
        applySetpoint1();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSim0();
        updateMotorSim1();
    }

    // triggers factories
    public Trigger motor0AtSetpoint(LinearVelocity velocitySetpoint, LinearVelocity velocitySetpointTolerance) {
        return new Trigger(() -> motor0Velocity.isNear(velocitySetpoint, velocitySetpointTolerance));
    }

    public Trigger motor1AtSetpoint(LinearVelocity velocitySetpoint, LinearVelocity velocitySetpointTolerance) {
        return new Trigger(() -> motor1Velocity.isNear(velocitySetpoint, velocitySetpointTolerance));
    }

    public Trigger motorsAtSetpoints(
            LinearVelocity motor0Setpoint,
            LinearVelocity motor1Setpoint,
            LinearVelocity motor0SetpointTolerance,
            LinearVelocity motor1SetpointTolerance) {
        return motor0AtSetpoint(motor0Setpoint, motor0SetpointTolerance)
                .and(motor1AtSetpoint(motor1Setpoint, motor1SetpointTolerance));
    }

    public Command createStop() {
        return runOnce(this::stop).withName("Stop");
    }

    public Command createApplySetpoint(String name, LinearVelocity motor0Setpoint, LinearVelocity motor1Setpoint) {
        return runOnce(() -> {
            this.motor0Setpoint.mut_replace(motor0Setpoint);
            this.motor1Setpoint.mut_replace(motor1Setpoint);
        }).withName(name);
    }


}

