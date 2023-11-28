package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends SubsystemBase {
    DcMotorEx intake;
    ServoEx holder;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Intake(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        holder = new SimpleServo(hardwareMap, "holder", 0, 180, AngleUnit.DEGREES);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hold();
    }

    public void setPower( double power){
        intake.setPower(power);
    }
    public void hold(){
        holder.turnToAngle(2);
    }

    public void leave(){
        holder.turnToAngle(155);
    }

}
