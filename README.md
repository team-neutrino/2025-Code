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
|           | 2              | 32     | ?                |
|           | 3              | 33     | ?                |

## DIO
| Subsystem | Description          | Port   |
| --------- | -------------------- | ------ |

## PWM
| Subsystem | Description          | Port   |
| --------- | -------------------- | ------ |

## PDH
| Port | Destination |
| ---- | ----------- |
| 0    | FLS         |
| 1    | FRS         |
| 2    | FLA         |
| 3    | FRA         |
| 4    |             |
| 5    |             |
| 6    |             |
| 7    |             |
| 8    |             |
| 9    |             |
| 10   |             |
| 11   |             |
| 12   |             |
| 13   |             |
| 14   |             |
| 15   |             |
| 16   | BLA         |
| 17   | BRA         |
| 18   | BLS         |
| 19   | BRS         |