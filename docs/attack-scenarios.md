# Attack Scenarios

This document explains the controlled CAN-layer attack scenarios used in the project.

---

## 1. Throttle Injection

The system sends unauthorised throttle control frames to demonstrate how injected CAN messages can affect acceleration behaviour.

**Purpose:** Show how a control message can override or interfere with normal behaviour.

---

## 2. Brake Injection

The system sends braking-related CAN frames to demonstrate unsafe interference with normal driving.

**Purpose:** Show how injected messages can affect braking behaviour.

---

## 3. Replay Attack

Previously captured CAN messages are replayed back onto the bus or into the control system.

**Purpose:** Show that valid messages can still be dangerous when reused at the wrong time.

---

## 4. Fuzzing

Unexpected or abnormal values are sent in CAN frames.

**Purpose:** Test how the system reacts to invalid or unusual input.

---

## 5. Denial of Service Behaviour

High-rate CAN traffic is generated to simulate message flooding.

**Purpose:** Demonstrate how excessive traffic can affect monitoring, logging, or system reliability.

---

## Safety Note

These demonstrations are only intended for a controlled academic simulation/testbed environment.
