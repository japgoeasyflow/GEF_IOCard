This **IOCARD connects Modbus RS485 instruments and actuators to a LoRaWAN network using EU868**. It reads measurements, sends them wirelessly, receives commands, and can switch equipment power through external Modbus relays.

| Function | What the card does |
|---|---|
| **Read instruments** | Stores up to **20 reading slots** for coils, digital inputs, holding registers, or input registers. |
| **Automatic reporting** | Reads configured slots and uploads measurements at a configurable interval. The default is **10 minutes**. |
| **Remote control** | Receives LoRaWAN commands to switch coils or write single/multiple Modbus registers. |
| **Fast data capture** | Captures repeated readings from one or two selected slots into memory, then uploads numbered batches. |
| **Instrument power saving** | Turns on an external power relay, waits for instruments to stabilize, collects readings, then turns the relay off. |
| **Actuator sequence** | Powers the configured equipment, writes a requested setpoint, activates the actuator, waits for movement, reads instruments, sends results, then switches power off sequentially. |
| **Save configuration** | Retains reading slots, reporting interval, and power-sequence settings in EEPROM across restarts. |
| **Network recovery** | Includes heartbeat checks, radio recovery, and automatic LoRaWAN rejoining. |

It operates in **LoRaWAN Class C**, listening for downlinks between transmissions. Commands arriving during a relay sequence are queued, with room for **four pending commands**.

For example, a remote command can request a valve position: the card powers the control equipment, sends the configured raw setpoint, powers the actuator, waits for movement, reports the resulting measurements, and shuts down the equipment power.

**Modbus writes are transmitted without waiting for acknowledgment**, so the card does not independently confirm that a relay switched or an actuator reached its position. That confirmation must come from the configured feedback readings.
