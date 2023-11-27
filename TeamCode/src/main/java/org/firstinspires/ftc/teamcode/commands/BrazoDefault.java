package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Brazo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class BrazoDefault extends CommandBase {
    Brazo brazo;
    Intake intake;
    GamepadEx gamepad;

    public BrazoDefault(Intake intake, GamepadEx gamepad){
        this.intake = intake;
        this.gamepad = gamepad;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>.5){
            CommandScheduler.getInstance().schedule(new InstantCommand(intake::hold));
        }



        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>.5){
            CommandScheduler.getInstance().schedule(new InstantCommand(intake::leave));
        }
    }
}
