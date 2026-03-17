# 🤖 EV3 Line Follower + Can Search


**LEGO Mindstorms EV3 robot** programmed in Python (ev3dev) that:
- Follows a line using a **PID controller** with two colour sensors
- Detects the **end of the line** using a stable white-white condition
- Runs an **interruptible ultrasonic arc scan** to search for a can/object
- **Returns to the line** using motor encoders, then resumes line following


> 📍 Multi-Robot Systems Project — University of Southern Denmark (SDU), 2025


---


## 🎥 Demo


▶️ [Robot Demo Video (Google Drive)](https://drive.google.com/file/d/1GZlPw8dLiJ200GplVoijCfNCUHn04S2z/view?usp=drive_link)


---


## ✨ Key Features


- ✅ **PID line following** (P/I/D) with integral clamping to avoid windup
- ✅ **Gap vs. end-of-line logic** using white threshold and sensor similarity checks
- ✅ **Ultrasonic arc scan** to search for a can/object at end of line
- ✅ **Encoder-based return** to line after search
- ✅ **Interruptible scan** — aborts immediately if line is detected during scan


---


## 🧱 Hardware Setup


| Component | Port |
|---|---|
| Left motor | `outA` |
| Right motor | `outD` |
| Color sensor (left) | `in3` → COL-REFLECT |
| Color sensor (right) | `in4` → COL-REFLECT |
| Ultrasonic sensor | `in2` → US-DIST-CM |


---


## 📁 Project Structure


```
ev3-line-follower-can-search/
├── README.md
├── LICENSE
├── src/
│   └── main.py          ← Main robot program (PID + can search)
├── docs/
│   └── report-ev3.pdf   ← Full project report
└── videos/
    └── links.md         ← Demo video links
```


---


## ⚙️ How to Run (ev3dev)


```bash
# Copy to EV3
scp -r ev3-line-follower-can-search robot@ev3dev.local:~/


# SSH into EV3
ssh robot@ev3dev.local


# Run
cd ev3-line-follower-can-search
python3 src/main.py
```


---


## 🔧 Key Parameters


| Parameter | Description |
|---|---|
| `baseline` | Base motor duty cycle |
| `P_GAIN / I_GAIN / D_GAIN` | PID gains for line following |
| `soglia_bianco` | White threshold (tune to your track) |
| `CAN_MIN_CM / CAN_MAX_CM` | Distance range to detect can |
| `ARC_WIDE` | Total ultrasonic scan arc in degrees |
| `K_TURN` | Turning calibration (degrees → ticks) |


---


## 🩹 Troubleshooting


| Problem | Solution |
|---|---|
| Oscillations on line | Reduce `P_GAIN`, increase `D_GAIN` |
| False end-of-line | Increase `LINE_LOST_LIMIT`, adjust `soglia_bianco` |
| Can not detected | Adjust `CAN_MIN_CM` / `CAN_MAX_CM` |
| Inaccurate turns | Recalibrate `K_TURN` and `SCALE` |


---


## 👤 Author


**Amirhossein Taleshinosrati**
- 🔗 GitHub: [github.com/Amirhossein7717](https://github.com/Amirhossein7717)
- 💼 LinkedIn: [linkedin.com/in/amirhossein-taleshinosrati](https://linkedin.com/in/amirhossein-taleshinosrati)
- 📧 amirhossein.taleshii@gmail.com


**Teammates:** Mounir Abbary · and team @ SDU Denmark


---


## 🧾 License


Licensed under the [MIT License](LICENSE).
