package org.firstinspires.ftc.teamcode.Mechanizmai;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//Dabar kuriant nauja Java Klase gali i si kreiptis su sitomis kodo linijomis:
//public HardwareInit hw;
//hw = new HardwareInit();
//hw.initializeHardware(hardwareMap);
//taip pat is naujo nereikia aprasyti Motoru, servu ar Dcmotoru, kreiptis su hw.(variable_name), pvz: hw.kP, hw.sm1, hw.pak0
public class HardwareInit {
    
    public Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    public DcMotor sm1, sm2; //Paėmimas, išmetimas //0, 1, 2expansion hub
    public DcMotor pad, pem;
    public Servo pak0, pak1; //Pakelimo servas Eh = 0, Eh 1
    public void initializeHardware(HardwareMap hardwareMap)
    {
        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub

        /*kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub*/

        kP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Išmetimas/Paėmimas

        sm1 = hardwareMap.get(DcMotor.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotor.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub

        /// Pak4limas
        pak0 = hardwareMap.get(Servo.class, "pak0");
        pak1 = hardwareMap.get(Servo.class, "pak1");

        pak0.setDirection(Servo.Direction.REVERSE);
        pak1.setDirection(Servo.Direction.REVERSE);
    }

}
