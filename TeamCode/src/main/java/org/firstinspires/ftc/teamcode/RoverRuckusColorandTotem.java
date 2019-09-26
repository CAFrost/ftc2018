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

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name="RoverRuckusColorAndTotem", group="2018")
public class RoverRuckusColorandTotem extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeft = null, motorDriveRight = null, motorLift = null, motorArm = null;

    private Servo servoClawLeft = null;
    private Servo servoClawRight = null;
    private Servo servoArm = null;
    //private Servo servoSensor = null;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private int startingPosition;
    private boolean driveBackwards , turnleft, driveForward, dropTotem, resetEncoderTurnLeft, resetEncoderDriveForward, raiseLift, leaveDepot, resetEncoderBackwards, turnRight,resetEncoderTurnRight, readcolor,readcolor2;
    private boolean resetEncoderColor, resetEncoderColor2, driveForward2, resetEncoderForward, turnRight2, resetEncoderRight;
    private int startingPositionDrive, startingPositionTurnLeft, startingPositionBackwards, startingPositionColor, startingPositionColor2, startingPositionForward, startingPositionTurnRight, startingPositionRight;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorDriveLeft  = hardwareMap.get(DcMotor.class, "m0");
        motorDriveRight  = hardwareMap.get(DcMotor.class, "m1");
        motorLift  = hardwareMap.get(DcMotor.class, "m2");
        motorArm  = hardwareMap.get(DcMotor.class, "am0");
        motorDriveLeft.setDirection((DcMotor.Direction.REVERSE));
        servoClawLeft = hardwareMap.get(Servo.class, "s0");
        servoClawRight = hardwareMap.get(Servo.class, "s1");
        closeClaw();
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servoArm = hardwareMap.get(Servo.class, "s2");
       // servoSensor = hardwareMap.get(Servo.class, "s3");
        //sensorColor = hardwareMap.get(ColorSensor.class, "cs0");
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "cs0");

        startingPosition = motorLift.getCurrentPosition();
        startingPositionDrive = motorDriveLeft.getCurrentPosition();
        //servoSensor.setPosition(0.9);
        driveBackwards = false;
        leaveDepot = false;
        turnleft = false;
        dropTotem = false;
        raiseLift = true;
        resetEncoderDriveForward = false;
        resetEncoderTurnLeft = false;
        turnRight = false;
        readcolor = false;
        readcolor2 = false;
        resetEncoderColor = false;
        resetEncoderColor2 = false;
        driveForward2 = false;
        resetEncoderForward = false;
        turnRight2 = false;
        resetEncoderRight = false;
        resetEncoderTurnRight = false;
        sensorColor = hardwareMap.get(ColorSensor.class, "cs");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "cs");


        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;

        leftPower = 0.5;
        rightPower = 0.5;
        telemetry.addData("State", "RaiseLift (%b)", raiseLift);
        telemetry.addData("State", "driveBackwards (%b)", driveBackwards);
        telemetry.addData("State", "turnLeft (%b)", turnleft);
        telemetry.addData("State", "driveForward (%b)", driveForward);
        telemetry.addData("State", "dropTotem (%b)", dropTotem);
        telemetry.addData("State", "leaveDepot (%b)", leaveDepot);



        if (raiseLift == true) {
            int position = motorLift.getCurrentPosition();
            motorLift.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Motors", "starting (%d), current (%d)", startingPosition, position);
            if (position < (startingPosition + 6900)) {
                //position = motorLift.getCurrentPosition();
                motorLift.setPower(0.4);
            } else {
                motorLift.setPower(0);
                driveBackwards = true;
                raiseLift=false;
            }
        }

        if (driveBackwards == true){
            int motorPosition = motorDriveLeft.getCurrentPosition();
            telemetry.addData("Motors", "starting (%d), current (%d)", startingPosition, motorPosition);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            if (motorPosition < (startingPositionDrive + 1000)) {
                motorDriveLeft.setPower(leftPower);
                motorDriveRight.setPower(rightPower);
            } else {
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                driveBackwards = false;
                turnleft = true;
                resetEncoderTurnLeft = true;

            }

        }
        if (turnleft == true) {
            if (resetEncoderTurnLeft == true) {
                startingPositionTurnLeft = motorDriveLeft.getCurrentPosition();
                resetEncoderTurnLeft = false;
            }
            int motorPositionLeft = motorDriveLeft.getCurrentPosition();
            telemetry.addData("TurnLeft", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("TurnLeft", "current (%d), starting (%d)", motorPositionLeft, startingPositionTurnLeft);

            if (motorPositionLeft < (startingPositionTurnLeft + 1300)) {
                motorDriveLeft.setPower(leftPower);
                motorDriveRight.setPower(-rightPower);
            } else {
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                resetEncoderDriveForward = true;
                driveForward = true;
                turnleft = false;

            }
        }
        if (driveForward == true){
            if (resetEncoderDriveForward == true){
                startingPositionDrive = motorDriveRight.getCurrentPosition();
                resetEncoderDriveForward = false;
            }
            int motorPositionForward = motorDriveRight.getCurrentPosition();
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Forward", "current (%d), starting (%d)", motorPositionForward, startingPositionDrive);

            if (motorPositionForward > (startingPositionDrive - 1000)) {
                motorDriveLeft.setPower(-leftPower);
                motorDriveRight.setPower(-rightPower);
            } else {
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                driveForward = false;
                turnRight = true;
                resetEncoderTurnRight = true;

            }


        }
        if (turnRight == true){
            if (resetEncoderTurnRight = true){
                startingPositionRight = motorDriveLeft.getCurrentPosition();
                resetEncoderTurnRight = false;
            }
            int motorPositionTurnLeft = motorDriveLeft.getCurrentPosition();
            if (motorPositionTurnLeft > (startingPositionRight - 2500)){
                motorDriveLeft.setPower(leftPower);
                motorDriveRight.setPower(-rightPower);
            }
            else{
                turnRight = false;
                motorDriveRight.setPower(0);
                motorDriveLeft.setPower(0);
                readcolor = true;
            }
        }
        if (readcolor == true){
            if (sensorColor.blue() <= 35 && sensorDistance.getDistance(DistanceUnit.CM) > 0)
            {
                resetEncoderColor = true;
                if (resetEncoderColor = true){
                    startingPositionColor = motorDriveLeft.getCurrentPosition();
                    resetEncoderColor = false;
                }
                int motorPositionColor = motorDriveLeft.getCurrentPosition();
                if (motorPositionColor < (startingPositionTurnLeft + 2500)){
                    motorDriveLeft.setPower(leftPower);
                    motorDriveRight.setPower(-rightPower);
                }
                else{
                    driveForward2 = true;
                    resetEncoderForward = true;
                }
                telemetry.addData("sensor","Gold");
            }
            else if (sensorColor.blue() > 35 && sensorDistance.getDistance(DistanceUnit.CM) > 0)
            {
                int motorPositionColor = motorDriveLeft.getCurrentPosition();
                if (motorPositionColor > (startingPositionTurnLeft - 2500)){
                    motorDriveLeft.setPower(leftPower);
                    motorDriveRight.setPower(rightPower);
                }else{
                    readcolor2 = true;
                }
                telemetry.addData("sensor", "Silver") ;
            }
            else {
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                readcolor = false;
            }
        }
        if (readcolor2 == true){
            if (sensorColor.blue() <= 35 && sensorDistance.getDistance(DistanceUnit.CM) > 0)
            {
                resetEncoderColor2 = true;
                if (resetEncoderColor2 = true){
                    startingPositionColor2 = motorDriveLeft.getCurrentPosition();
                    resetEncoderColor2 = false;
                }
                int motorPositionColor2 = motorDriveLeft.getCurrentPosition();
                if (motorPositionColor2 < (startingPositionTurnLeft + 2500)){
                    motorDriveLeft.setPower(leftPower);
                    motorDriveRight.setPower(-rightPower);
                }
                else {
                    driveForward2 = true;
                    resetEncoderRight = true;
                }
                telemetry.addData("sensor","Gold");
            }
            else if (sensorColor.blue() > 35 && sensorDistance.getDistance(DistanceUnit.CM) > 0)
            {
                int resetColor2 = motorDriveLeft.getCurrentPosition();
                if (resetColor2 > (startingPositionTurnLeft - 3000)){
                    motorDriveLeft.setPower(-leftPower);
                    motorDriveRight.setPower(-rightPower);
                }
                else {
                    turnRight2 = true;
                    resetEncoderRight = true;
                }
                telemetry.addData("sensor", "Silver") ;
            }
            else {
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                readcolor2 = false;
            }
        }
        if (turnRight2 == true){
            if (resetEncoderRight){
                startingPositionRight = motorDriveLeft.getCurrentPosition();
                resetEncoderRight = false;
            }
            int TurnRight2 = motorDriveLeft.getCurrentPosition();
            if (TurnRight2 < (startingPositionTurnRight + 2500)){
                motorDriveRight.setPower(rightPower);
                motorDriveLeft.setPower(-leftPower);
            }
            else{
                driveForward2 = true;
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                turnRight2 = false;
            }
        }
        if (driveForward2 == true){
            if (resetEncoderForward){
                startingPositionForward = motorDriveLeft.getCurrentPosition();
                resetEncoderForward = false;
            }
            int driveForward = motorDriveLeft.getCurrentPosition();
            if (driveForward > (startingPositionForward - 2500 )){
                motorDriveRight.setPower(rightPower);
                motorDriveLeft.setPower(leftPower);
            }
            else{
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                resetEncoderForward = false;
                driveForward2 = false;
                dropTotem = true;
            }
        }

        if (dropTotem == true){
            liftClaw();
            openClaw();
            dropTotem = false;
            leaveDepot = true;
            resetEncoderBackwards = true;
            //servoClawLeft.setPosition(0);
            //servoClawRight.setPosition(1);
        }
        if (leaveDepot == true){
            if (resetEncoderBackwards == true){
                startingPositionBackwards = motorDriveRight.getCurrentPosition();
                resetEncoderBackwards = false;

            }
            int MotorPositionBackwards = motorDriveRight.getCurrentPosition();

            if (MotorPositionBackwards < startingPositionBackwards + 3000){
                motorDriveRight.setPower(rightPower);
                motorDriveLeft.setPower(leftPower);
            }else{
                motorDriveLeft.setPower(0);
                motorDriveRight.setPower(0);
                leaveDepot = false;
            }

        }





        // drive backwards

        // lower lift
        //motorLift.setDirection((DcMotor.Direction.FORWARD));



    }

    public void closeClaw(){
        servoClawLeft.setPosition(0.5);
        servoClawRight.setPosition(0.5);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 0, 1);
    }


    public void liftClaw(){
        servoArm.setPosition(.5);
    }

    public void openClaw(){
        servoClawLeft.setPosition(0.2);
        servoClawRight.setPosition(0.8);
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
