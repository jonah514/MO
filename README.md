# MO (From WALL-E)

A FreeRTOS-based firmware for an ESP32 Feather V2 that supports three operation modes (Random, Remote, Voice), real-time ultrasonic sensing, servo/motor control, and a diagnostic stream for LabVIEW.

---

## 1. Hardware & Wiring

- **Ultrasonic Sensor (HC-SR04–compatible)**
  - **VCC** → 3V3
  - **GND** → GND
  - **TRIG** → GPIO4
  - **ECHO** → GPIO15
- **Servos**
  - Servo 1 → GPIO12 (LEDC channel 0)
  - Servo 2 → GPIO13 (LEDC channel 1)
- **Motors (H-bridge inputs)**
  - A_IN1 → GPIO7
  - A_IN2 → GPIO8
  - B_IN3 → GPIO26
  - B_IN4 → GPIO25
- **Heartbeat / Expression LED** → GPIO13 (shared with Servo 2 signal, used as status LED)

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
| `3`     | **Mode → Voice**: (placeholder) voice task runs               |
| `F`     | **Forward** (Remote mode only)                                |
| `G`     | **Reverse** (Remote mode only)                                |
| `L`     | **Turn Left** (Remote mode only)                              |
| `H`     | **Turn Right** (Remote mode only)                             |
| `S`     | **Stop Motors** (Remote mode only)                            |
| `U`     | **On-demand Ultrasonic**: replies `"DIST xx.x\r\n"`           |
| (Periodic) | **Diagnostics** (every 1 s): `"OBSTCNT <n> DIST <x.x>\r\n"` |

---

## 4. FreeRTOS Tasks & Priorities

| Task                | Priority | Purpose                                                                                  |
|:-------------------:|:--------:|:-----------------------------------------------------------------------------------------|
| **echo_task**       | **10**   | UART manager & mode switch; interprets LabVIEW commands, suspends/resumes tasks         |
| **random_task**     | **5**    | Automatic navigation: drives forward, checks distance, reacts and turns on obstacle      |
| **voice_task**      | **4**    | Placeholder for future voice-control logic (starts suspended until Mode 3)               |
| **diagnostic_task** | **3**    | Sends obstacle count & latest distance over UART every second                           |
| **Timer ISR**       | n/a      | Hardware timer callback every 500 ms → updates `g_ultrasonic_cm` (preempts all tasks)   |

- **`TaskHandle_t`**  
  - Captures a reference when you need to suspend/resume/delete a task.  
  - E.g. `randomTaskHandle` is used by `echo_task` to start/stop random navigation.  
  - We pass `NULL` for `diagnostic_task` since it runs continuously and never needs to be paused.

---

## 5. Ultrasonic & Interrupts

- An **ESP-Timer** is configured in `app_main()` to fire **every 500 ms**.  
- Its callback (`ultrasonic_timer_cb`) runs in **ISR context**, pulsing the TRIG pin, measuring ECHO timing, and storing the result in `g_ultrasonic_cm`.  
- Because ISRs preempt all tasks, the distance is updated without blocking any FreeRTOS task.

---

## 6. Obstacle Reaction & Diagnostics

- **`random_task`** monitors `g_ultrasonic_cm`.  
  - If distance < 20 cm, increments `obstacle_count`, calls `express_react()` (LED flashes + servo wave), then picks a random turn.  
- **`diagnostic_task`** wakes every 1 s and emits:
  - OBSTCNT <obstacle_count> DIST <g_ultrasonic_cm>
  over UART for LabVIEW to parse and display.

---

## 7. Multitasking & Scheduling Logic

1. **UART manager** (prio 10) always preempts other tasks to handle commands in real time.  
2. **Random navigation** (prio 5) preempts diagnostics but yields during delays.  
3. **Voice placeholder** (prio 4) suspended until Mode 3 selected.  
4. **Diagnostics** (prio 3) runs only when higher-priority tasks are blocked/yielding.  
5. **Timer ISR** (hardware level) fires every 500 ms to update the ultrasonic reading, potentially unblocking the random task immediately if in obstacle threshold.

_FreeRTOS_ ensures that the highest-priority ready task runs at all times, while timer interrupts and GPIO ISRs can wake or notify tasks to guarantee responsive, real-time behavior.