package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Brazo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pinza;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveSubsystem;

@TeleOp
public class TeleOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepadDriver = new GamepadEx(gamepad1);
        GamepadEx gamepadX = new GamepadEx(gamepad2);

        SampleTankDrive sampleTankDrive = new SampleTankDrive(hardwareMap);
        TankDriveSubsystem driveSystem = new TankDriveSubsystem(sampleTankDrive);
        /*Intake intake = new Intake(telemetry, hardwareMap);
        Brazo brazo = new Brazo(telemetry, hardwareMap);
        Pinza pinza = new Pinza(telemetry, hardwareMap);

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(()-> brazo.setBrazo(brazo.getBrazoPos()+20, 1))
                .whenReleased(()-> brazo.setBrazo(brazo.getBrazoPos(), 0));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(()-> brazo.setBrazo(brazo.getBrazoPos()-20, 1))
                .whenReleased(()-> brazo.setBrazo(brazo.getBrazoPos(), 0));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> brazo.setBrazo(brazo.getBrazoPos()-10, 1));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()-> brazo.setBrazo(brazo.getBrazoPos()+10, 1));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X)
                .whenPressed(()-> pinza.agarrar());

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(()-> pinza.soltar());

         */




        driveSystem.setDefaultCommand(new TankDriveCommand(
                driveSystem, () -> -gamepadDriver.getLeftY(), gamepadDriver::getRightX
        ));

        schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading", driveSystem.getPoseEstimate().getHeading());
            telemetry.update();
        }));
    }
}
