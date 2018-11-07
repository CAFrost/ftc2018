/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RoverRukus", group="2018")
public class RoverRukus extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeft = null, motorDriveRight = null, motorLift = null, motorArm = null;

    private Servo servoClawLeft = null;
    private Servo servoClawRight = null;
    private Servo servoArm = null;
    private Servo servoSensor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorDriveLeft  = hardwareMap.get(DcMotor.class, "m0");
        motorDriveRight  = hardwareMap.get(DcMotor.class, "m1");
        motorLift  = hardwareMap.get(DcMotor.class, "m2");
        motorArm  = hardwareMap.get(DcMotor.class, "m3");

        servoClawLeft = hardwareMap.get(Servo.class, "s0");
        servoClawRight = hardwareMap.get(Servo.class, "s1");
//        closeClaw();
        servoArm = hardwareMap.get(Servo.class, "s2");
        servoSensor = hardwareMap.get(Servo.class, "s3");
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;
        double armPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -0.9, 0.9) ;
        rightPower   = Range.clip(drive - turn, -0.9, 0.9) ;

        // Send calculated power to wheels
        motorDriveLeft.setPower(leftPower);
        motorDriveRight.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        boolean clawIsOpen=false;
        //boolean
        boolean buttonOpen=gamepad2.a;
        boolean buttonClose=gamepad2.b;
        boolean buttonOpen60=gamepad2.y;
        boolean buttonOpen40=gamepad2.x;
        boolean buttonArmUp=gamepad1.x;
        boolean buttonArmDown=gamepad1.y;
        if (buttonArmUp || buttonArmDown)
        {
            int position = motorLift.getCurrentPosition();
            telemetry.addData("Encoder Position", position);

            if (buttonArmUp) //if (motorLift.getCurrentPosition() <= 0)
            {
                motorLift.setDirection(DcMotor.Direction.FORWARD);
            }
            //else if (motorLift.getCurrentPosition() >= 1000)
            //{
            //     motorLift.setPower(0);
           // }
            if (buttonArmDown) //if (motorLift.getCurrentPosition() <=1224)
            {
                motorLift.setDirection((DcMotor.Direction.REVERSE));
            }
            //else if (motorLift.getCurrentPosition() <=0)
             //   motorLift.setPower(0);
            motorLift.setPower(1);
        }
        else
        {
            motorLift.setPower(0);
        }
        telemetry.addData("Buttons", "Open (%b), Close (%b)", buttonOpen, buttonClose);

        if (buttonClose)
        {
            closeClaw();
        }
        else if (buttonOpen)
        {
            openClaw();
        }
        else if (buttonOpen60)
        {
            openClaw60();
        }
        else if (buttonOpen40)
        {
            openClaw40();
        }
    }

    public void closeClaw(){
        servoClawLeft.setPosition(0);
        servoClawRight.setPosition(1);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 0, 1);
    }

    public void liftArm(){

        telemetry.addData("Arm",  "Up");
    }

    public void lowerArm(){

        telemetry.addData("Arm", "Down");
    }
    public void openClaw(){
        servoClawLeft.setPosition(1);
        servoClawRight.setPosition(0);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 1, 0);
    }

    public void openClaw60(){
        servoClawLeft.setPosition(0.25);
        servoClawRight.setPosition(0.75);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 1, 0);
    }

    public void openClaw40(){
        servoClawLeft.setPosition(0.50);
        servoClawRight.setPosition(0.50);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 1, 0);
    }

    @Override
    public void stop() {
        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
        motorLift.setPower(0);
        motorArm.setPower(0);
    }
}
