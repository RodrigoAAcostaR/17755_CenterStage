package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends SubsystemBase {
    ServoEx caja;
    DcMotorEx roller;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Intake(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        caja = new SimpleServo(hardwareMap, "caja", 0, 180, AngleUnit.DEGREES);
        //roller = hardwareMap.get(DcMotorEx.class, "roller");


    }

    public void hold(){
        caja.turnToAngle(0);
    }

    public void leave(){
        caja.turnToAngle(180);
    }


    public void setRoller(double power){
        roller.setPower(power);
    }

}
