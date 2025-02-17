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
| Climb     | Servo for ratchet    | 0      |

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
| 7    | LL, Pi, LED | 20 A    | 18 AWG |
| 8    |             |         |        |
| 9    | Switch      | 10 A    | 22 AWG |
| 10   | Intake 1    | 30 A    | 14 AWG |
| 11   |             |         |        |
| 12   |             |         |        |
| 13   | Arm         | 40 A    | 12 AWG |
| 14   | Elevator 1  | 40 A    | 12 AWG |
| 15   | Elevator 2  | 40 A    | 12 AWG |
| 16   | BRA         | 40 A    | 12 AWG |
| 17   | FRA         | 40 A    | 12 AWG |
| 18   | BRS         | 40 A    | 8 AWG  |
| 19   | FRS         | 40 A    | 8 AWG  |
| ---- | ----------- | ------- | ------ |
| 20   | Canandcolor | 3 A     | 18 AWG |
| 21   | RPM         | 10 A    | 22 AWG |
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

## Buttons Controller
| Button     | Function       |
| ---------- | -------------- |
| Y          | Score Coral L1 |
| X          | Score Coral L2 |
| B          | Score Coral L3 |
| A          | Score Coral L4 |
| LT         | Intake Coral   |
| LB         | Intake Coral   |
| RT         |                |
| RB         |                |
| DU         |                |
| DD         |                |
| DL         |                |
| DR         |                |
| LSB        |                |
| RSB        |                |
| Start      |                |
| Back       |                |


## Driver Controller
| Button     | Function        |
| ---------- | --------------- |
| Y          | Raise Climb Arm |
| X          | Lock            |
| B          | Unlock          |
| A          | Lower Climb Arm |
| LT         |                 |
| LB         | Reset Climb Arm |
| RT         | Slow swerve     |
| RB         | Reset Lock      |
| DU         |                 |
| DD         |                 |
| DL         |                 |
| DR         |                 |
| LSB        |                 |
| RSB        |                 |
| Start      |                 |
| Back       | Reset Yaw       |

## Raspberry Pi
contains code intended to be run on an FRC robot's coprocessor

``` bash

pip install pynetworktables
pip install ntcore # don't acutally need this
pip3 install rpi_ws281x adafruit-circuitpython-neopixel
python3 -m pip install --force-reinstall adafruit-blinka
pip install adafruit-board-toolkit

add sudo and --break-system-packages to all pip installs
(ex. sudo pip install pynetworktables --break-system-packages)
python main.py 192.168.88.135 # robot or simulator IP address

## Auton Paths
 LEAVE for all starting line areas (Top, Middle, Processor)
 L1 CORAL TOP-Scores 1 coral on L1 from the top of the starting line to ID19
 L1 CORAL MIDDLE-Scores 1 coral on L1 from the middle of the staring line to ID21
 L1 CORAL PROCESSOR-Scores 1 coral on L1 from the processor side of the starting line to ID17
 2 CORAL TOP-Scores 1 coral on L1 from the top of the starting line to ID19, intake closest human playerstation, then score L1 to ID19 again 
 2 CORAL MIDDLE-Scores 1 coral on L1 from the top of the starting line to ID17, intake closest human playerstation, then score L1 to ID17 again
 2 CORAL PROCESSOR-Scores 1 coral on L1 from the top of the starting line to ID17, intake closest human playerstation, then score L1 to ID17 again
 2 CORAL TOP(Score then score at 18)-Scores 1 coral on L1 from the top of the starting line to ID19, intake closest human playerstation, score L1 to ID18
 2 CORAL PROCESSOR(Score the score at 18)-Scores 1 coral on L1 from the top of the starting line to ID17, intake closest human playerstation, score L1 to ID18
 3 CORAL MIDDLE-Starts at middle of starting line goes back and forth from ID17 and human player 3 times, only scores L1

