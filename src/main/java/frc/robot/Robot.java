// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;


public class Robot extends TimedRobot {

    private final CommandXboxController controller = new CommandXboxController(0);

    public Robot() {
        CommandIntake commandIntake = CommandIntakeGen.create();
        CommandClaw commandClaw = CommandClawGen.create();
        // controller.a().onTrue(commandClaw.createToggle0());
        // controller.b().onTrue(commandClaw.createToggle1());
        // controller.x().onTrue(commandClaw.createToggle2());

        PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        SmartDashboard.putData("Power Distribution", pdh);

        RobotModeTriggers.teleop().onTrue(commandIntake.createStop());
        controller.a()
                .whileTrue(commandIntake.createApplyVelocitySetpoint(
                        "15 RadPerSec",
                        RadiansPerSecond.of(100)))
                .onFalse(commandIntake.createStop());
        controller.b()
                .whileTrue(commandIntake.createApplyPositionSetpoint(
                        "50 Radians",
                        Radians.of(50.0)))
                .onFalse(commandIntake.createStop());
        controller.x()
                .whileTrue(commandIntake.createApplyPositionSetpoint(
                        "0 Radians",
                        Radians.of(0.0)
                ))
                .onFalse(commandIntake.createStop());


        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
        addPeriodic(commandClaw::periodic, 0.020);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        addPeriodic(commandIntake::periodic, 0.020);
        if (RobotBase.isSimulation()) {
            addPeriodic(commandIntake::simulationPeriodic, 0.01);
        }


    }

}
