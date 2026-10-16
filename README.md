<h1 align="center">
  <br>
  <img src="https://github.com/RaviKachhwaha/RUDRAKINUXIS-Lite/blob/main/docs/images/RUDRAKINUXIS_Lite_3D_front_image.png" alt="docs/images/RUDRAKINUXIS_Lite_3D_front_image.png">

  <br>
  RUDRAKINUXIS-Lite
  <br>
</h1>
<h3 align="center">
A fully open-sourced complete Field Oriented Control ESC. 
</h3>

| Front Render | Back Render | 
| --- | --- |
| ![](https://github.com/RaviKachhwaha/RUDRAKINUXIS-Lite/blob/main/docs/images/RUDRAKINUXIS_Lite_3D_front_image.png) | ![](https://github.com/RaviKachhwaha/RUDRAKINUXIS-Lite/blob/main/docs/images/RUDRAKINUXIS_Lite_3D_back_image.png) | 

## Schematics  

Here are the schematic design,

![](https://github.com/RaviKachhwaha/RUDRAKINUXIS-Lite/blob/main/docs/images/RUDRAKINUXIS_Lite_schematic.png)

## Features
- **STM32G431CBU6 MCU** - Arm Cortex-M4 with CORDIC accelerator, running at 170 MHz.
- **A 24kHz 6S FOC motor controller and real-time DSP**.
- **Texas Instruments DRV8300 gate driver**.
- **600W continuous with 16 AWG bus wire and 2oz copper support**.
- Power and arming status indicator LEDs.
- High-speed CAN FD and DShot support.
- **USB C** - for flashing and telemetry.
- A 20-25.2V (6S LiPo) pad input for power.
- **3-phase 0.5mΩ low side shunts** - with dedicated Kelvin differential sensing.
- **LMR36510 synchronous buck converter** - high efficiency step down for the 10V gate rail.
- **AP2112K ultra low noise 3.3V LDO** - delivers clean, ripple free power to the MCU ADC.
- **Phase BEMF and bus voltage sensing** - for sensorless motor tracking and battery telemetry.
- **Onboard NTC thermistor** - for real time thermal monitoring and overheat protection.
- **USBLC6-2SC6 ESD protection** - hardware transient voltage clamping on USB data lines.
- **SWD debug header** - 5-pin port for direct ST-Link firmware flashing and live debugging.

## PCB Design 
| ![](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/docs/images/pcb_layout_2d..png) |
|--- |  


 ### The PCB is 4 layers with dimensions 000x00mm, made to be easily useable for anyone.  

 ### All the PCB files are [here](https://github.com/RaviKachhwaha/RUDRAKINUXIS-Lite/blob/main/hardware) 

| Front Layer | Inner Layer 1 |
| --- | --- |
| ![](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/docs/images/pcb_layout_top_layer..png) | ![](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/docs/images/pcb_layout_2nd_layer..png) |
| Inner Layer 2 | Bottom layer |
| ![](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/docs/images/pcb_layout_3nd_layer..png) | ![](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/docs/images/pcb_layout_bottom_layer..png) |

## Pinout  

This is a general pinout 

| ![](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/docs/images/Pin_Outs..png) | 
| --- | 

(Credits to KiCad EDA for KiCad 10, STMicroelectronics for the STM32G4 MCU, Texas Instruments for the DRV8300 gate driver, and Hack Club for funding to make this possible.)

(Thanks to Madhav, tty7 to review my Design and tell me some mistakes like the antenna is pleased on board without open air due to that the range of it is low after the review from them I places the antenna in open air to get full range of Bluetooth and correct Wi-Fi CSI) 

## Why was it made?  

While using the normal old ESC for my many projects and in some times need, I realised it doesn't have Field Oriented Control and high power support and I couldn't use it for high power and control needed project, unfortunately I had to think to buy a different high power ESC then I see that those have high power but not Field Oriented Control support or many have Field Oriented Control but cost very high and now have my needed pins and full control over every thing and use very low price components which give low quality, but now I designed one with the simplicity of STM32G431CBU6 with being able to get best Field Oriented Control and fastest processing and control over every thing and also programming support in a same board with high quality Field Oriented Control for best flight and have too many features and many things possible.

## How you can get one for yourself? 

That is pretty easy, you have all the files you will need in the production folder, order the development  
board from any PCB fab ( I chose Lion Circuits or PCB Power cause its cheap in India and has good quality and also made in India ) and BINGO! You will have it to use for any purpose you want!  

## How do you use it?  
You can use it as any normal ESC board but first want to flash the firmware which is given in firmware folder, for flashing first connect the ESC to your computer via USB-C. Open your flashing tool (STM32CubeProgrammer or stm32flash), select the CP2102N COM port, and click **Download / Flash**. The onboard auto-reset circuit automatically enters the STM32 system bootloader, flashes the binary, and boots directly into your firmware and you are good to go!
I have attached a simple firmware to start with prebuild firmware available in the firmware folder.  

## BOM  
|Name|Purpose          |Quantity|Total Cost (USD)|Link               |Distributor|
|----|-----------------|--------|----------------|-------------------|-----------|
|PCBA|Board and Assembly|2       |62.12           |[Lion Circuits](lioncircuits.com)|Lion Circuits     |
|Electronic Components|BOM Component Sourcing|2       |117.00           |[Lion Circuits](lioncircuits.com)|Lion Circuits     |
|2 Components|2 BOM Component Sourcing which is not got from Lion Circuits|2       |2.59           |[Sharvi Electronics](sharvielectronics.com)|Sharvi Electronics     |
|Battery|18650 3.7V 3000mAh Li-ion Battery with Connector for testing|1       |5.00          |[18650 Battery](https://www.amazon.in/gp/product/B0HBPWY1CS/ref=ox_sc_act_title_3?smid=AJWV8HAM5L3YL&psc=1)|Amazon India     |
|Speaker|5 Inch 4 Ohm 25W Full-Range Woofer Speaker (Stereo Pair) for testing|1       |5.48           |[5 Inch Speaker](https://www.amazon.in/gp/product/B0D8CVNZVK/ref=ox_sc_act_title_2?smid=A3C7Z4RVM9A3G1&psc=1)| Amazon India     |
|Data & Power Cable|100W USB-C to USB-C Silicone Cable (3ft / 1.0m) for uploading firmware|1       |5.00           |[USB-C Cable](https://www.amazon.in/gp/product/B0F316KC1Q/ref=ox_sc_act_title_4?smid=AJ6SIZC8YQDZX&psc=1)| Amazon India     |
|Speaker connection Cable|20 Meter speaker wiring Cable (64ft / 20.0m) for connecting the speakers and testing|1       |3.10           |[20 Meter Wire](https://www.amazon.in/gp/product/B0CHBFB5H2/ref=ox_sc_act_title_1?smid=A2LH9DQUAZIHP2&psc=1)| Amazon India     |
|Aux Cable|5 Meter Aux Cable for testing the 3.5 mm Aux jack and wired audio input system|1       |2.70           |[5 Meter Aux Cable](https://www.amazon.in/gp/product/B08XBFV4VR/ref=ox_sc_act_title_2?smid=A2LH9DQUAZIHP2&psc=1)| Amazon India     |
|Power Supply|20 Volt Power supply to run the board on its high power to test all things is working on its full power|1       |5.55           |[20V Power Supply](https://www.amazon.in/gp/product/B0D22W62W9/ref=ox_sc_act_title_1?smid=A2QFGJBWFSGOIU&psc=1)| Amazon India     |
|Cable to give power to power supply|1.5 Meter Power Cable to give power to power supply adapter due to it not have cable with it|1       |1.60           |[Main Power cable](https://www.amazon.in/gp/product/B0F9VD5NC5/ref=ox_sc_act_title_1?smid=AJ6SIZC8YQDZX&psc=1)| Amazon India     |
|3D Printing|3D printing of cases of board only|2       |17.00           |[3D Printing Service](https://3ding.in)|3Ding    |
| | | Total | 227.85 |

LCSC & DigiKey BOM for the components on board is [here](https://github.com/RaviKachhwaha/AuraForge-50X/blob/main/components_bom.csv)

!!Thanks to ALL!!
