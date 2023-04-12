# Power Distribution Module (PDM) Firmware
## Overview
The Power Distribution Module (PDM) safely regulates and distributes power to the rest of the vehicle. It takes in power from a custom made 5S lithium ion battery pack with an output voltage range of 15-21V. Since many high power devices on the vehicle require 12V, the PDM has a set of 4x 12V 20A switching regulators which supply power for those devices (Air Cooling Fans, Liquid Cooling Fan, Liquid Cooling Pump, and the vehicle 12 volt rail). Since some devices on the vehicle are capable of taking in the 15-21V directly from the battery while others are only 5V tolerant, those two voltage ranges are provided as well. 

For safety, all outputs and inputs have current monitoring capabilities, fusing, and can be disabled by the ESP32 MCU except for the main power input which is enabled by grounding the main kill pin. The core idea is that the MCU will be monitoring the current output so if an output is shorted then the MCU can disable that output immediately without any irreversible damage. If this monitoring fails, the fuse on the output of that individual regulator will blow, resulting in the device no longer being powered but the rest of the PDM continuing to operate as intended. If that fuse fails as well, then the 2x25A fuse on the main power input will blow before the battery is damaged, but the PDM will lose power. As one final step to ensure the battery is never damaged, the battery also comes installed with a BMS equipped with overcurrent protection.
## TODO
- Log sensor values on CAN 
    - Add frames to DBC file 
- Migrate main to timer-driven w/ CAN signals 
- Ramp up turning on devices via PWM to avoid the large current spike
- Disable output if current exceeds desired current for that output for a given duration 
## Technical Resources 
- [High Level Overview](https://docs.google.com/document/d/1Hcizixf9vJc9UTDhetP1IpuLdgTR1PF8x2v2It-QL1k/edit?usp=sharing)
- [Power Delivery Requirements](https://docs.google.com/spreadsheets/d/1fbh53vEgTszFUH2TGYJv4j2e1aOxvuQcn50ekbZ68nU/edit?usp=sharing)
- [MCU Pin Mapping](https://docs.google.com/spreadsheets/d/1zyGeT-fVR49wVKpqhybv1xxE6zk0hX3oLBnuzNhSnwU/edit?usp=sharing) 
- [Output Pinout](https://docs.google.com/spreadsheets/d/1okobf2fIMFn24W8cS57rP1c5T_ALurxeMwLw3RJwlyc/edit?usp=sharing)