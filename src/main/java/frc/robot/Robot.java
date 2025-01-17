// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandIntake;
import frc.robot.subsystems.CommandIntakeGen;

import static edu.wpi.first.units.Units.*;


public class Robot extends TimedRobot {

    private final CommandXboxController controller = new CommandXboxController(0);

    public Robot() {
        CommandIntake commandIntake = CommandIntakeGen.create();

        RobotModeTriggers.teleop().onTrue(commandIntake.createStop());
        controller.a()
                .whileTrue(commandIntake.createApplySetpoint(
                        "500 RPM",
                        MetersPerSecond.of(10),
                        MetersPerSecond.of(10)))
                .onFalse(commandIntake.createStop());

        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
        addPeriodic(commandIntake::periodic, 0.01);
        if (RobotBase.isSimulation()) {
            addPeriodic(commandIntake::simulationPeriodic, 0.01);
        }

    }

}
