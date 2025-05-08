# MO (From WALL-E)

A FreeRTOS-based firmware for an ESP32 Feather V2 that supports three operation modes (Random, Remote, Voice), real-time ultrasonic sensing, servo/motor control, LCD display for expressive eyes, and a diagnostic stream for LabVIEW.

---

## 1. Hardware & Wiring

- **Ultrasonic Sensor (HC-SR04–compatible)**
  - **VCC** → 3V3
  - **GND** → GND
  - **TRIG** → GPIO21
  - **ECHO** → GPIO36
- **Servos**
  - Servo 1 (Tilt) → GPIO4 (LEDC channel 0)
  - Servo 2 (Pan) → GPIO5 (LEDC channel 1)
  - Servo 3 (Arm) → GPIO13 (LEDC channel 2)
- **Motors (H-bridge inputs)**
  - A_IN1 → GPIO7
  - A_IN2 → GPIO8
  - B_IN3 → GPIO26
  - B_IN4 → GPIO25
- **Microphone**
  - Signal → GPIO34 (ADC1 Channel 6)
- **LCD Display (16x2)**
  - RS → GPIO14
  - E → GPIO32
  - D4 → GPIO15
  - D5 → GPIO33
  - D6 → GPIO27
  - D7 → GPIO12
- **Status LED** → GPIO13 (shared with Servo 3 signal)

---

## 2. Build & Flash

1. Open in VS Code with the ESP-IDF extension.  
2. Set SERIAL PORT to the port of the ESP32 Feather V2.  
3. **Build** & **Flash**
4. Open **MO SERIAL.vi** in LabView, **Run** the VI, and press **RESET** on the ESP32.

---

## 3. LabVIEW Command Map

| Command | Action                                                       |
|:-------:|:-------------------------------------------------------------|
| `I`     | **Handshake**: LED13 ON, replies `"ESP32\r\n"`               |
| `1`     | **Mode → Random**: obstacle-avoidance task runs               |
| `2`     | **Mode → Remote**: manual motor/servo commands enable         |
| `3`     | **Mode → Voice**: voice detection task runs                   |
| `F`     | **Forward** (Remote mode only)                                |
| `G`     | **Reverse** (Remote mode only)                                |
| `L`     | **Turn Left** (Remote mode only)                              |
| `H`     | **Turn Right** (Remote mode only)                             |
| `S`     | **Stop Motors** (Remote mode only)                            |
| `U`     | **On-demand Ultrasonic**: replies `"DIST xx.x\r\n"`           |
| `7`     | **Raise Arm**: moves arm servo up                             |
| `8`     | **Lower Arm**: moves arm servo down                           |
| `X`     | **Express Anger**: triggers anger animation (in Random mode)  |
| (Periodic) | **Diagnostics** (every 1 s): `"OBSTCNT <n>\r\n"`           |

---

## 4. FreeRTOS Tasks & Priorities

| Task                | Priority | Purpose                                                                                  |
|:-------------------:|:--------:|:-----------------------------------------------------------------------------------------|
| **echo_task**       | **10**   | UART manager & mode switch; interprets LabVIEW commands, suspends/resumes tasks         |
| **microphone_task** | **8**    | Monitors microphone input for sound detection and triggers reactions                     |
| **random_task**     | **5**    | Automatic navigation: drives forward, checks distance, reacts and turns on obstacle      |
| **voice_task**      | **4**    | Placeholder for future voice-control logic (starts suspended until Mode 3)               |
| **diagnostic_task** | **3**    | Sends obstacle count over UART every second                                             |
| **Timer ISR**       | n/a      | Hardware timer callback every 200 ms → updates `g_ultrasonic_cm` (preempts all tasks)   |

- **`TaskHandle_t`**  
  - Captures a reference when you need to suspend/resume/delete a task.  
  - E.g. `randomTaskHandle` is used by `echo_task` to start/stop random navigation.  
  - We pass `NULL` for `diagnostic_task` since it runs continuously and never needs to be paused.

---

## 5. Ultrasonic & Interrupts

- An **ESP-Timer** is configured in `app_main()` to fire **every 200 ms**.  
- Its callback (`ultrasonic_timer_cb`) runs in **ISR context**, pulsing the TRIG pin, measuring ECHO timing, and storing the result in `g_ultrasonic_cm`.  
- Because ISRs preempt all tasks, the distance is updated without blocking any FreeRTOS task.

---

## 6. LCD Display & Expressions

- The 16x2 LCD display is used to create expressive eyes for MO.
- Custom characters are defined for:
  - Full blocks (normal eyes)
  - Empty blocks (closed eyes)
  - Middle lines (half-closed eyes)
- Functions for different expressions:
  - `display_normal_eyes()`: Default square eyes
  - `blink_eyes()`: Natural blinking animation
  - `display_angry_eyes()`: Currently uses normal eyes (placeholder)

---

## 7. Obstacle Reaction & Expressions

- **`random_task`** monitors `g_ultrasonic_cm`.  
  - If distance < 20 cm, increments `obstacle_count`, backs up, turns randomly, then calls `express_react()`.
  - If `obstacle_count` > 5, triggers `express_anger()` animation.
- **`express_react()`**: LED flashes, arm raises, eyes blink, servos wave.
- **`express_anger()`**: Robot spins in circles, arm raises, resets obstacle count.
- **`microphone_task`**: Detects loud sounds and triggers reactions.

---

## 8. Multitasking & Scheduling Logic

1. **UART manager** (prio 10) always preempts other tasks to handle commands in real time.
2. **Microphone task** (prio 8) monitors sound levels and can interrupt movement for reactions.
3. **Random navigation** (prio 5) preempts diagnostics but yields during delays.
4. **Voice placeholder** (prio 4) suspended until Mode 3 selected.
5. **Diagnostics** (prio 3) runs only when higher-priority tasks are blocked/yielding.
6. **Timer ISR** (hardware level) fires every 200 ms to update the ultrasonic reading.

_FreeRTOS_ ensures that the highest-priority ready task runs at all times, while timer interrupts and GPIO ISRs can wake or notify tasks to guarantee responsive, real-time behavior.