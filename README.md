# can2040-rust

This is a port of [can2040 in C](https://github.com/KevinOConnor/can2040). Based on this, we can enable CAN on RP2040 with Rust.

## Run CAN demo

### Demo devices
- RP2040 dev board
- Waveshare SN65HVD230 board (CAN-board): [[Waveshare SN65HVD230](https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO/ref=sr_1_2?crid=2I4ZLTIPIB93Q&keywords=SN65HVD230+waveshare&qid=1696911860&sprefix=sn65hvd230+waveshar%2Caps%2C146&sr=8-2)]
- USB-CAN Adapter (USB): [[Amazon](https://www.amazon.com/PRIZOM-Converter-Debugger-Analyzer-Candlelight/dp/B0CD6QFQXH/ref=sr_1_6?crid=2TGJJD1KV2Z36&keywords=CANable&qid=1696911666&sprefix=canable%2Caps%2C353&sr=8-6&th=1)]

### Wiring
- RP2040 GPIO 26 <=> CAN RX
- RP2040 GPIO 27 <=> CAN TX
- RP2040 GND <=> CAN GND
- RP2040 3V3 <=> CAN 3V3

- CAN CAN-H <=> USB CAN-H
- CAN CAN-L <=> USB CAN-L

### Run can2040_demo
The demo is run on Linux, using a relatively low baud rate of 10_000. After debugging, you can choose a higher baud rate yourself.

#### Make USB ready 
Plug the USB into your computer and make sure the corresponding socket is enabled (here it corresponds to socket can0).
```shell
sudo ip link set down can0 && \
sudo ip link set can0 type can bitrate 10000 && \
sudo ip link set up can0
```

Then, run the following command to monitor the messages on the CAN bus for debugging:
```shell
candump can0
```

#### Run Can2040_demo on RP2040.
Make sure all the wiring is correct, and if you adjust the GPIOs, make sure the corresponding GPIOs in the code are adjusted as well. Once everything is ready, you can run the following command, and you should see RP2040 continuously sending CAN frames to the USB:

```shell
cargo run --release --example can2040_demo
```

#### Send to test the receive
Run the following command on your computer and make sure the RP2040 console window receives the information:
```shell
cansend can0 123#DEADBEEF12345678 
```

### Some tips
- A logic analyzer can be connected to CAN-H / L in differential mode to obtain the correct CAN signals. The connection method is to connect CAN-H to the signal and CAN-L to GND.
- A dual-channel oscilloscope can also be connected to CAN-H/L to view the differential signal.
- Global common ground is not required. Even if the obtained signal is interfered with, SN65HVD230 can still extract the normal CAN signal for TX/RX.
  