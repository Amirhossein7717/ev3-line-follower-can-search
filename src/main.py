#!/usr/bin/env python3
"""
EV3 Line Follower + Can Search (Ultrasonic Arc Scan)
Author : Amirhossein Taleshinosrati
Team   : Multi-Robot Systems Project - SDU, Denmark (2025)
         (Mounir Abbary, Nicolas Lambropoulos, and team)

Hardware:
    Left motor: outA  |  Right motor: outD
    Color sensor left: in3  |  Color sensor right: in4  |  Ultrasonic: in2

Usage:
    python3 src/main.py
"""
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from ev3dev2.sensor import INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.button import Button
import time

mL = LargeMotor(OUTPUT_A)
mR = LargeMotor(OUTPUT_D)
clLeft  = ColorSensor(INPUT_3)
clRight = ColorSensor(INPUT_4)
us      = UltrasonicSensor(INPUT_2)
btn     = Button()
clLeft.mode = clRight.mode = 'COL-REFLECT'
us.mode = 'US-DIST-CM'

baseline=30; P_GAIN=1.2; I_GAIN=0.05; D_GAIN=0.8; INTEGRAL_LIM=50
soglia_bianco=60; CONTRAST_EPS=10; LINE_LOST_LIMIT=8
CAN_MIN_CM=5; CAN_MAX_CM=30; ARC_WIDE=180; ARC_WIDE_STEP=10
K_TURN=5.6; WHEEL_DIAM_MM=56.0; DRIVE_SIGN=1; SCALE=1.0; dt=0.02

def turn_deg(deg):
    t=int(deg*K_TURN)
    mL.run_to_rel_pos(position_sp=t, speed_sp=200, stop_action='brake')
    mR.run_to_rel_pos(position_sp=-t,speed_sp=200, stop_action='brake')
    mL.wait_while('running'); mR.wait_while('running')

def drive_cm(cm):
    t=int((cm*10/(3.14159*WHEEL_DIAM_MM))*360)
    mL.run_to_rel_pos(position_sp=DRIVE_SIGN*t,speed_sp=200,stop_action='brake')
    mR.run_to_rel_pos(position_sp=DRIVE_SIGN*t,speed_sp=200,stop_action='brake')
    mL.wait_while('running'); mR.wait_while('running')

def stop_motors():
    mL.stop(stop_action='brake'); mR.stop(stop_action='brake')

def is_white(v): return v > soglia_bianco
def sees_line(): return not is_white(clLeft.value()) or not is_white(clRight.value())

def search_can():
    print("[SEARCH] Starting arc scan...")
    sl,sr=mL.position,mR.position
    drive_cm(5); time.sleep(0.2)
    samples=[]; half=ARC_WIDE//2; turn_deg(-half)
    for a in range(-half, half+1, ARC_WIDE_STEP):
        d=us.value()/10.0; samples.append((a,d))
        print(f"  angle={a:4d} dist={d:.1f}cm")
        if sees_line(): print("[SEARCH] Line detected - aborting!"); break
        turn_deg(ARC_WIDE_STEP); time.sleep(0.05)
    win=[(a,d) for a,d in samples if CAN_MIN_CM<=d<=CAN_MAX_CM]
    if win:
        ca=sum(a for a,_ in win)/len(win)
        print(f"[SEARCH] Can detected at {ca:.1f} deg")
        turn_deg(ca-samples[-1][0]); drive_cm(3); time.sleep(0.5)
    else: print("[SEARCH] No can found.")
    mL.run_to_rel_pos(position_sp=int((sl-mL.position)*SCALE),speed_sp=300,stop_action='brake')
    mR.run_to_rel_pos(position_sp=int((sr-mR.position)*SCALE),speed_sp=300,stop_action='brake')
    mL.wait_while('running'); mR.wait_while('running')
    print("[SEARCH] Resuming line follow.")

def main():
    print("=== EV3 Line Follower + Can Search ===")
    integral=prev_err=lost=0.0; mL.reset(); mR.reset()
    while not btn.any():
        lv=clLeft.value(); rv=clRight.value()
        if is_white(lv) and is_white(rv) and abs(lv-rv)<CONTRAST_EPS:
            lost+=1
            if lost>=LINE_LOST_LIMIT:
                print("[EOL] End of line!"); stop_motors()
                drive_cm(-3); search_can(); lost=integral=prev_err=0.0; continue
        else: lost=0
        err=float(lv-rv)
        integral=max(-INTEGRAL_LIM,min(INTEGRAL_LIM,integral+err*dt))
        corr=P_GAIN*err+I_GAIN*integral+D_GAIN*(err-prev_err)/dt
        prev_err=err; mc=baseline*1.5; corr=max(-mc,min(mc,corr))
        mL.run_direct(duty_cycle_sp=int(baseline+corr))
        mR.run_direct(duty_cycle_sp=int(baseline-corr))
        time.sleep(dt)
    stop_motors(); print("=== Stopped ===")

if __name__=='__main__': main()
