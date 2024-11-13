# EZ-USB&trade; FX2G3: USB video class (UVC) application

This application implements a UVC 1.1-compliant camera application using the EZ-USB&trade; FX2G3 device. An integrated USB audio class interface is provided to stream the audio data received from a PDM microphone. This code example shows the configuration and usage of the sensor interface port (SIP) on the FX2G3 device to implement the Synchronous Slave FIFO IN protocol.

> **Note:** This code example is an alpha release only for EZ-USB&trade; FX2G3 devices.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-fx2g3-uvc-uac)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyNDA2MzciLCJTcGVjIE51bWJlciI6IjAwMi00MDYzNyIsIkRvYyBUaXRsZSI6IkVaLVVTQiZ0cmFkZTsgRlgyRzM6IEhlbGxvIHdvcmxkIiwicmlkIjoic3VrdSIsIkRvYyB2ZXJzaW9uIjoiMS4wLjAiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiV0lSRUQiLCJEb2MgRmFtaWx5IjoiVVNCX0hTIn0=)


## Requirements
- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.2 or later (tested with v3.2)
- Board support package (BSP) minimum required version: 4.3.2
- Programming language: C
- Associated parts: [EZ-USB&trade; FX2G3](https://www.infineon.com/cms/en/product/promopages/ez-usb-fx2g3/)


## Supported toolchains (make variable 'TOOLCHAIN')
- GNU Arm&reg; Embedded Compiler v11.3.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.16 (`ARM`)

## Supported kits (make variable 'TARGET')

- [EZ-USB&trade; FX2G3 DVK](https://github.com/Infineon/mtb-example-fx2g3-uvc-uac) (`KIT_FX2G3_104LGA`) – Default value of `TARGET`


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).

This example requires no additional software or tools.


## Using the code example


### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b. Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[EZ-USB&trade; FX2G3: USB video class (UVC) application](https://github.com/Infineon/mtb-example-fx2g3-uvc-uac)" application with the desired name "FX2G3 USB UVC" configured for the *KIT_FX2G3_104LGA* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id KIT_FX2G3_104LGA --app-id mtb-example-fx2g3-uvc-uac --user-app-name FX2G3 USB UVC --target-dir "C:/mtb_projects"
   ```


The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>

### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board (J2) to your PC using the provided USB cable. Connect the USBFS port (J7) on the board to PC for debug logs.

2. Open a terminal program and select the Serial COM port. Set the serial port parameters to 8N1 and 921600 baud.

3. Follow these steps to program the board using the **EZ-USB&trade; FX Control Center** (Alpha) application.
      
   1. Open **EZ-USB&trade; FX Control Center** application.

      The **EZ-USB&trade; FX2G3** device displays as **EZ-USB FX BOOTLOADER**.

   2. Program the FPGA binary on to the external flash with the following steps:
   
      a. Navigate to **Device Selection** > **Devices**, select **EZ-USB FX BOOTLOADER**, and then click the **Program** > **Internal Flash** option.

      b. Browse the [mtb-example-fx2g3-flash-loader application] (https://github.com/Infineon/mtb-example-fx2g3-flash-loader), compile, and navigate to the */build/APP_KIT_FX2G3_104LGA/Release/* folder within the CE directory and locate the *.hex* file and program.

      c. Select the FX2G3 flash loader device in **EZ-USB&trade; FX Control Center**.

      d. Navigate to **Program** > **External Flash**.

      e. Browse the *FPGA binary* file in the *<CE title>/BitFile* folder based on the configuration.

   3. Once the FPGA binary programming is successful, return to USB bootloader mode. Once the firmware binary has been programmed onto the FX2G3 device flash, the bootloader will keep transferring control to the application on every subsequent reset.

   4. To return the control to USB bootloader, press the **BOOT MODE/PMODE (SW1)** switch on *KIT_FX2G3_104LGA DVK*.
      While the device is reset or power cycled, the device will stay in the bootloader mode instead of booting into the application.

   5. Select the **FX Bootloader** device in **EZ USB&trade; FX Control Center** and click **Program** > **Internal Flash**.

   6. Navigate to the *<CE title>/build/APP_KIT_FX2G3_104LGA/Release/* folder within the CE directory and locate the *.hex* file, and program.

      Confirm if the programming is successful in the log window of the application.

4. After programming, the application starts automatically. Confirm that "\<CE Title>" is displayed on the UART terminal.

   **Figure 1. Terminal output on program startup**

   ![](images/terminal-fx2g3-uvc-uac.png)

5. Open any third party camera application, select the FX2G3 device and video resolution to stream the video. 

   By default, the device will stream 1K (1920x1080 ~15fps) video data.


## Logging configurations

By default, the USBFS port is enabled for debug logs.
To enable debug logs on UART, set **USBFS_LOGS_ENABLE** compiler flag to '0u' in the *makefile*. SCB4 of the FX2G3 device is used as UART with a baud rate of 921,600 to send out log messages through the P11.0 pin.


## Debugging

Debug the code example by setting debug levels for the UART logs. Set the **DEBUG_LEVEL** macro in *main.c* file with the following values for debugging.

**Table 2. Debug values**

Macro value     |    Description
:-------------  | :------------
1u              | Enable error messages
2u                  | Enable warning messages
3u              | Enable info messages
4u                | Enable all messages

<br>


## Design and implementation

This code example demonstrates how to implement USB video class and audio class specifications, letting the FX2G3 function as a UVC- and UAC-compliant composite device. This enables video and audio data to be streamed over USB 2.0. This application uses various low-performance peripherals to interface with the system such as:

- I2C master to configure the video source
- PDM receiver interface to connect audio source
- SMIF (in x1 or single mode) interface for downloading the FPGA configuration binary on every bootup
- Enable debug prints over CDC using the USBFS block on FX2G3


### Features of the application

- **USB specifications:** USB 2.0 (Hi-Speed)
> **Note:** UVC and UAC functions are not supported in USB Full-Speed connections.
- **Video Format:** Uncompressed YUY2 (YUYV)
- **Video Resolutions:** USB2.0 (HS): 1920X1080 (1K), 640X480 (VGA)
- Supports video streaming using PCAM v2 image sensor

This code example currently does not implement any UVC control functions such as brightness, contrast, or exposure etc.
   
This application demonstrates:
   
- The usage of the FX2G3 APIs to implement a standard USB video class device. Handling of class specific USB control requests at the application level.
- Streaming video data at USB HS speed (1K Video at ~15 fps and VGA video stream at ~60 fps) from the SIP to USB endpoint.
- FPGA configuration using the SMIF Block of FX2G3
- I2C register writes to configure FPGA register


### Video/audio streaming data path


#### Audio streaming

- The application enables EP 1-IN as Isochronous endpoints with a maximum packet size of 192 bytes
- The device receives the audio data through the PDM receiver and sends it to EP 3-IN
- Four 192-byte DMA buffers are used to hold the data while it is being forwarded to USB


#### Video streaming

- The application enables EP 1-IN as BULK endpoints with a maximum packet size of 512 bytes
- The device receives the video data through LVCMOS socket 0 and sends it on EP 1-IN
- Four 61440-byte DMA buffers are used to hold the data while it is being forwarded to USB


#### UVC header addition modes 

A 32-byte UVC header is added to each buffer containing 61408 bytes of video data and the resulting 61440 bytes are sent to the USB host. By default, the header addition is performed by a firmware task.

If the FPGA that drives the SIP interface of FX2G3 supports header addition, the application can be configured to enable FPGA to add a UVC header, thereby speeding up the streaming operation.

- The firmware adds 32 bytes of UVC header in the data received from the LVCMOS side before the data is sent to the USB side on the UVC BULK endpoint 1-IN
- UVC header is inserted FPGA itself. To enable this feature, enable the PRE_ADDED_HEADER compiler flag in the *usb_app.h*

> **Note:** This example does not support UVC header insertion using the LVCMOS IP. This option can only be used if the FPGA that provides the video data supports "enhanced mode" commands to trigger the UVC header (metadata) addition. This feature will be available in next release.


### Application workflow

The application flow involves three steps - Initialization, USB device enumeration, and external flash programming.


#### Initialization

During initialization, the following steps are performed:

1. All the required data structures are initialized.
2. USBD and USB driver (CAL) layers are initialized.
3. Application registers all descriptors supported by function/application with the USBD layer.
4. Application registers callback functions for different events like RESET, SUSPEND, RESUME, SET_CONFIGURATION, SET_INTERFACE, SET_FEATURE, and CLEAR_FEATURE. USBD will call the respective callback function when the corresponding events are detected.
5. Initialize the data transfer state machines.
6. Application registers handlers for all relevant interrupts.
7. Application makes the USB device visible to the host by calling the Connect API.
8. FPGA is configured using the SMIF (in x1 or single mode) block to read the bit file stored in the external flash. FPGA sees the data on bus and gets configured.
9. FPGA is initialized using I2C writes to FPGA registers.
10. Application initializes the SIP block on FX2G3 as required by the selected LVCMOS operating mode.


#### USB device enumeration

1. During USB device enumeration, the host requests for descriptors which are already registered with the USBD layer during the initialization phase.
2. Host will send SET_CONFIGURATION command and SET_INTERFACE commands to activate required function in the device.
3. After SET_CONFIGURATION and SET_INTERFACE command, the application task takes control and enables the endpoints for data transfer.


#### UVC data transfer

- Depending on compile time options, LVCMOS Interface, 16-bit bus width, and UVC header addition by FX2G3 are selected
- Once the UVC camera is opened in the host application: 

   - The streaming DMA channel is enabled
   
   - The video source is configured to stream the selected video resolution 
   
   - The DMA ready flag on the SIP interface is asserted 
   
   - The FPGA data source starts streaming video data to the FX2G3 device

- Video data moves from the LVCMOS subsystem to the SRAM through high-Bandwidth DMA
- The data is forwarded to the USBHS EP 1-IN. DataWire DMA channels are used for USBHS transfers
- Video data moves from USB device BULK endpoint 1-IN to the HOST UVC application


#### Integrated UAC data transfer

In addition to the USB video camera interface, this application also implements a USB audio class interface, which carries the data received from a PDM microphone.

The P9.0 and P9.1 pins of the FX2G3 device are used to interface to the PDM microphone.

- P9.0 is the clock output from the FX2G3 device, which is generated at 3.072 MHz
- P9.1 is the data input from the PDM microphones to the FX2G3 device

A pair of microphones in stereo configuration can be connected. Selection between mono and stereo microphone configuration is done using the STEREO_ENABLE pre-processor setting, which can be updated through the *makefile*. By default, the interface is configured for mono operation. 


### Compile-time configurations

This application's functionality can be customized through the compile-time parameters that can be turned ON or OFF through the *makefile*.
The application uses the GNU Arm&reg; 11.3 toolchain, which is part of the ModusToolbox&trade; installation for compilation.
- Run the `make` command to compile the application and generate a USB bootloader compatible binary. This binary can be programmed to the FX2G3 device using the EZ-USB&trade; Control Center application.
- Run the `make BLENABLE=no` command to compile the application and generate the standalone binary. This binary can be programmed onto the FX2G3 device through the SWD interface using the OpenOCD tool. For more details, see the [EZ-USB&trade; FX2G3 SDK user guide](https://www.infineon.com/cms/en/product/promopages/ez-usb-fx2g3/#main-features-comparison).

By default, the application is configured to receive data from a 16-bit wide LVCMOS interface in SDR mode and make a USBHS data connection. These settings can be modified by passing additional options on the build command line or modifying settings in the *Makefile*.

**Table 3. Macro description**

Macro name        | Description                               | Allowed values
:-------------    | :------------                             | :--------------
BUS_WIDTH_16       | Select the LVCMOS bus width              | 1u for 16-bit <br> 0u for 8-bit bus width
INTERLEAVE_EN      | Enable GPIF thread interleaving        | 1u to enable <br> 0u to disable
PRE_ADDED_HEADER   | UVC header addition                      | 1u for FPGA added header <br> 0u for FX2G3 added header
AUDIO_IF_EN        | Enable Integrated UAC function           | 1u to enable <br> 0u to disable
STEREO_ENABLE      | Enable Audio in Stereo Mode             | 1u for stereo audio <br> 0u for mono audio
MIPI_SOURCE_ENABLE | Enable MIPI Video source                | 1u if MIPI source is present <br> 0u for FPGA-generated colorbar
USBFS_LOGS_ENABLE  | Enable debug logs through USBFS port     | 1u for debug logs over USBFS <br> 0u for debug logs over UART (SCB4)

<br>


## FPGA BitFile information

FPGA binary in the *BitFile* folder of the project can be programmed to an external flash on the FX2G3 DVK using the *mtb-example-fx2g3-flash-loader* firmware. 
Follow these steps to program the binary file to external flash:

1. Program *mtb-example-fx2g3-flash-loader.hex* using **EZ-USB&trade; FX Control Center**.
2. Click **Program** > **External Flash**. Browse the *FPGA binary* file.
3. Check the programming status.

**Table 4: BitFile description**

BitFile                                         |    Description   
:--------------------                           | :----------------                
*FX2G3_Design_PassiveX1_16Bit_LVCMOS_RX.bin*    | LVCMOS RX 16-Bit (default)          
*FX2G3_Design_PassiveX1_8Bit_LVCMOS_RX*         | LVCMOS RX 8-Bit

<br>


### Application files

**Table 5. Application file description**

File                                | Description   
:-------------                      | :------------                         
*gpif_header_lvcmos.h*              | Generated Header file for GPIF state configuration for LVCMOS Interface
*usb_app.c*                         | C source file implementing UVC 1.1 application logic
*usb_app.h*                         | Header file for application data structures and functions declaration
*usb_uvc_device.h*                  | Header file with UVC application constants and the video frame configurations
*usb_descriptors.c*                 | C source file containing the USB descriptors
*main.c*                            | Source file for device initialization, ISRs, LVCMOS interface initialization, etc.
*uac_app.c*                         | C source file with UAC interface handlers
*usb_i2c.c*                         | C source file with I2C handlers
*usb_i2c.h*                         | Header file with I2C application constants and the function definitions
*usb_qspi.c*                        | C source file with SMIF handlers and FPGA configuration functions
*usb_qspi.h*                        | Header file with SMIF application constants and the function definitions
*cm0_code.c*                        | CM0 initialization code
*usb_imagesensor.c*                 | C Source file for PCAM v2 image sensor configuration
*usb_imagesensor.h*                 | Header file for PCAM v2 image sensor
*Makefile*                          | GNU make compliant build script for compiling this example

<br>


## Related resources

Resources  | Links
-----------|----------------------------------
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [FX2G3 datasheets](https://www.infineon.com/cms/en/product/promopages/ez-usb-fx2g3/#!?fileId=8ac78c8c90530b3a01909c03f29537e0)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries on GitHub | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – Peripheral Driver Library (PDL) and docs
Middleware on GitHub  | [usbfxstack](https://github.com/Infineon/usbfxstack) – USBFXStack middleware library and docs
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.

> **Note:** For more information about the software modules and configuration options, see the [EZ-USB&trade; FX2G3 SDK user guide](https://www.infineon.com/cms/en/product/promopages/ez-usb-fx2g3/#main-features-comparison).
<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history


Document title: *CE240688* – *EZ-USB FX2G3: USB video class (UVC) application*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
<br>



All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.


---------------------------------------------------------

© Cypress Semiconductor Corporation, 2024. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
