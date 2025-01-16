// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandIntake;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.*;
import static edu.wpi.first.units.Units.*;


public class Robot extends TimedRobot {

    private final CommandXboxController controller = new CommandXboxController(0);

    public Robot() {
        double gearing = 12.0;
        Distance radius = Meters.of(1.0);

        EncoderConfig encoderConfig0 = new EncoderConfig()
                .positionConversionFactor(2 * Math.PI * radius.baseUnitMagnitude() / gearing)
                .velocityConversionFactor(2 * Math.PI * radius.baseUnitMagnitude() / gearing / 60.0)
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(16);
        EncoderConfig encoderConfig1 = new EncoderConfig()
                .positionConversionFactor(2 * Math.PI * radius.baseUnitMagnitude() / gearing)
                .velocityConversionFactor(2 * Math.PI * radius.baseUnitMagnitude() / gearing / 60.0)
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(16);

        DCMotor dcMotor0 = DCMotor.getNeo550(1);
        DCMotor dcMotor1 = DCMotor.getNeo550(1);

        double JKgMetersSquared0 = 1;
        double JKgMetersSquared1 = 1;

        LinearSystem<N2, N1, N2> linearSystem0 = LinearSystemId.createDCMotorSystem(dcMotor0, JKgMetersSquared0, gearing);
        LinearSystem<N2, N1, N2> linearSystem1 = LinearSystemId.createDCMotorSystem(dcMotor1, JKgMetersSquared1, gearing);

        DCMotorSim dcMotorSim0 = new DCMotorSim(linearSystem0, dcMotor0);
        DCMotorSim dcMotorSim1 = new DCMotorSim(linearSystem1, dcMotor1);

        double ks0 = 0.12;
        double ks1 = 0.12;
        double ka0 = 1.0 / linearSystem0.getB(1, 0);
        double ka1 = 1.0 / linearSystem1.getB(1, 0);
        double kv0 = -linearSystem0.getA(1, 1) * ka0;
        double kv1 = -linearSystem1.getA(1, 1) * ka1;

        SimpleMotorFeedforward motor0Feedforward = new SimpleMotorFeedforward(ks0, kv0, ka0, 0.01);
        SimpleMotorFeedforward motor1Feedforward = new SimpleMotorFeedforward(ks1, kv1, ka1, 0.01);
        double maxAcceleration0 = motor0Feedforward.maxAchievableAcceleration(12.0, 0.0);
        double maxAcceleration1 = motor1Feedforward.maxAchievableAcceleration(12.0, 0.0);


        ClosedLoopConfig closedLoopConfig0 = new ClosedLoopConfig()
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0.0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                .feedbackSensor(kPrimaryEncoder);
        ClosedLoopConfig closedLoopConfig1 = new ClosedLoopConfig()
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0.0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                .feedbackSensor(kPrimaryEncoder);

        SparkMax sparkMax0 = new SparkMax(16, kBrushless);
        SparkMax sparkMax1 = new SparkMax(18, kBrushless);
        SparkBaseConfig sparkBaseConfig0 = new SparkMaxConfig()
                .apply(encoderConfig0)
                .apply(closedLoopConfig0);
        SparkBaseConfig sparkBaseConfig1 = new SparkMaxConfig()
                .apply(encoderConfig1)
                .apply(closedLoopConfig1);
        sparkMax0.configure(sparkBaseConfig0, kResetSafeParameters, kPersistParameters);
        sparkMax1.configure(sparkBaseConfig1, kResetSafeParameters, kPersistParameters);



        SlewRateLimiter slewRateLimiter0 = new SlewRateLimiter(maxAcceleration0);
        SlewRateLimiter slewRateLimiter1 = new SlewRateLimiter(maxAcceleration1);
        CommandIntake commandIntake = new CommandIntake(
                sparkMax0,
                sparkMax1,
                motor0Feedforward,
                motor1Feedforward,
                slewRateLimiter0,
                slewRateLimiter1,
                dcMotorSim0,
                dcMotorSim1,
                dcMotor0,
                dcMotor1);

        RobotModeTriggers.teleop().onTrue(commandIntake.createStop());
        controller.a()
                .whileTrue(commandIntake.createApplySetpoint(
                        "500 RPM",
                        MetersPerSecond.of(50),
                        MetersPerSecond.of(50)))
                .onFalse(commandIntake.createStop());

        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
        addPeriodic(commandIntake::periodic, 0.01);
        if (RobotBase.isSimulation()) {
            addPeriodic(commandIntake::simulationPeriodic, 0.01);
        }

    }

}
