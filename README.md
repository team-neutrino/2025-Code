# 2025-Code

## Reefscape documentation

- [Game Manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf)
- [Full drawing package](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf)
- [Layout and Marking Diagram](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf)
- [April Tags](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf)

## Useful Java Docs

- [WPILIB](https://github.wpilib.org/allwpilib/docs/release/java/index.html)
- [Phoenix](https://api.ctr-electronics.com/phoenix6/release/java/)
- [Rev](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
- [LimeLight](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib)

## Rio Bus CAN IDs
| Subsystem | Description    | CAN ID | Node Type        |
| --------- | -------------- | ------ | ---------------- |
| Robot     |                |        |                  |
|           | RoboRIO        | 00     | RoboRIO          |
|           | PDH            | 01     | PDH              |
| Elevator  |                | 02-09  |                  |
|           | height control | 02     | Spark Flex       |
|           | height control | 03     | Spark Flex       |
| Arm       |                | 10-19  |                  |
|           | angle control  | 10     | Spark Flex       |
| Claw      |                | 20-29  |                  |
|           | left grabber   | 21     | Spark Max        |
|           | right grabber  | 22     | Spark Max        |
|           | wrist          | 25     | Spark Max        |
|           | color sensor   | 27     | Canandcolor      |
| Climb     |                | 30-39  |                  |
|           | 0              | 30     | Talon Fx         |
|           | 1              | 31     | Talon Fx         |
|           | hook           | 32     | Spark Max        |

## DIO
| Subsystem | Description          | Port   |
| --------- | -------------------- | ------ |

## PWM
| Subsystem | Description          | Port   |
| --------- | -------------------- | ------ |

## PDH
| Port | Destination | Breaker | Gauge  |
| ---- | ----------- | ------- | ------ |
| 0    | FLS         | 40 A    | 8 AWG  |
| 1    | BLS         | 40 A    | 8 AWG  |
| 2    | FLA         | 40 A    | 12 AWG |
| 3    | BLA         | 40 A    | 12 AWG |
| 4    | Climb 1     | 40 A    | 8 AWG  |
| 5    | Climb 2     | 40 A    | 8 AWG  |
| 6    | Climb Hook  | 30 A    | 14 AWG |
| 7    | Limelight   | 20 A    | 20 AWG |
| 8    | Switch      | 10 A    | 20 AWG |
| 9    |             |         |        |
| 10   | Intake 1    | 30 A    | 14 AWG |
| 11   | Intake 2    | 30 A    | 14 AWG |
| 12   | Wrist       | 30 A    | 14 AWG |
| 13   | Arm         | 40 A    | 12 AWG |
| 14   | Elevator 1  | 40 A    | 12 AWG |
| 15   | Elevator 2  | 40 A    | 12 AWG |
| 16   | BRA         | 40 A    | 12 AWG |
| 17   | FRA         | 40 A    | 12 AWG |
| 18   | BRS         | 40 A    | 8 AWG  |
| 19   | FRS         | 40 A    | 8 AWG  |
| ---- | ----------- | ------- | ------ |
| 20   | Canandcolor | 3 A     | 18 AWG |
| 21   | Pi & LEDs   | 10 A    | 18 AWG |
| 22   | VRM         | 20 A    | 18 AWG |
| 23   | RoboRIO     | 10 A    | 18 AWG |

## VRM
| Channel    | Destination |
| ---------- | ----------- |
| 12V/2A     |             |
|            | CANcoders   |
|            |             |
| 12V/500 mA |             |
|            | CANivore    |
|            | Pigeon 2.0  |
| 5V/2A      |             |
|            |             |
|            |             |
| 5V/500 mA  |             |
|            |             |
|            |             |
