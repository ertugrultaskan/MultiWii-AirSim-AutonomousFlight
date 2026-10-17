# Drone Simulation: MultiWii + AirSim Integration

This project connects the MultiWii flight controller running on an Arduino Nano with the AirSim simulation environment. A Python script acts as the bridge between the two systems, exchanging sensor data, waypoints, and motor commands to fly a simulated drone.

## System Architecture

```
AirSim (simulation)
   |  sensor data (IMU, GPS, etc.)
   v
Python script ---- waypoints ----> Arduino Nano (MultiWii)
   ^                                    |
   |<------- motor values --------------+   (MSP protocol)
   |
   v
Motor commands sent back to AirSim -> drone moves
```

- **AirSim:** Provides the simulated environment and drone, and generates sensor data.
- **Python script:** Forwards sensor data and waypoints to MultiWii, receives motor values, and applies them to the drone in AirSim.
- **MultiWii (Arduino Nano):** Runs the flight control algorithm and computes motor outputs.

## Status

- [ ] Designing System Architecture