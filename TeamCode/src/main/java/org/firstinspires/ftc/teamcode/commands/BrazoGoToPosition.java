package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Brazo;

public class BrazoGoToPosition extends CommandBase {
    Brazo brazo;
    int setPoint;

    public BrazoGoToPosition(Brazo brazo, int setPoint){
        this.brazo = brazo;
        this.setPoint = setPoint;

        addRequirements(brazo);
    }

    @Override
    public void execute() {
        brazo.setPosition(setPoint);
    }

    @Override
    public boolean isFinished() {
        return brazo.isAtSetpoint();
    }
}
