QF_AMAZON_ALEXA
===============

Amazom AVS Test requires connection to the Amazon Cloud using Amazon SDK.
Espressif LyraT Board has all the required Hardware and software SDK to support it.

QuickLogic AVS setup uses Espressif SDK modified to take the Audio Data from
QuickFeather board. So instead of LyraT board ESP-DevKitC-VE board can be 
used.

Espressif Tool Chain and SDK setup
==================================

1. Follow the steps from the following link to download and install Tool Chain

https://docs.espressif.com/projects/esp-idf/en/v3.2.2/get-started/windows-setup.html


Download: https://dl.espressif.com/dl/esp32_win32_msys2_environment_and_toolchain-20181001.zip (625MB)

    -Unzip the file to C:\Espressif\ or some other folder
    -It will create an \msys32 directory with a pre-prepared environment.
    -Run C:\Espressif\msys32\mingw32.exe
    -Make you own "user" directory under \home (optional)
    -Make a directory called "esp" under \home\<user>\
       mkdir -p ~/esp
    -change the directory path to esp
    
Note: All the ESP SDKs should be installed in \esp directory
There are 3 different SDKs that need to installed.
1.esp-idf SDK - this the basic sdk that is required for ESP32 projects
2.esp-adf SDK - This is Audio Development SDK
3.esp-va-sdk  - This is the Voice Assitant SDK for LyraT Boards

~~~~~~~~~~
Important:
~~~~~~~~~~
esp-va-sdk works with only v3.2 of esp-idf.

2. So, follow the instructions from the link below.

https://github.com/espressif/esp-va-sdk/blob/master/README-Getting-Started.md

-Install esp-idf v3.2
-Install esp-va-sdk
Do not build it now, since the SDK PATHS are not yet set

3. Next Install "esp-adf sdk only" following the instructions from the link below

https://docs.espressif.com/projects/esp-adf/en/latest/get-started/index.html#step-1-set-up-esp-idf

The above link is a section for the LyraT Guide

https://docs.espressif.com/projects/esp-adf/en/latest/get-started/get-started-esp32-lyrat.html

4. Rename the folder /esp-va-sdk/examples/amazon_alexa to "amazon_alexa_orginal" if you
   want to keep it as reference. But not required.

5. Copy the QuickLogic provided "qf_amazon_alexa" code to the folder /esp-va-sdk/examples/

6. Create a shell script file called "export_idf_paths.sh" in msys32/etc/profile.d/ directory

Add the following "export paths" to the script file.
1. Add path to esp-idf using the variable IDF_PATH
2. Add path to esp-adf using the variable ADF_PATH
3. Add path to Audio Board using the variable AUDIO_BOARD_PATH
  (Note: we use the DevKitC-VE board path later)
4. All export the COM port for the DevKitC-VE using the variable ESPPORT

See the example "export_idf_paths.sh" in this directory. Here are the contents for convenience

    ----------------------------------------------------------------
    export IDF_PATH="/home/<user>/esp/esp-idf"
    export ADF_PATH="/home/<user>/esp/esp-adf"
    export AUDIO_BOARD_PATH="/home/<user?/esp/esp-va-sdk/examples/qf_amazon_alexa/devkitc_ve/audio_board/audio_board_lyrat/"
    export ESPPORT=COM15
    -----------------------------------------------------------------
    
    Replace <user> with your "user name".
    
    Note that AUDIO_BOARD_PATH for LyraT board is replaced by a path that is part
    "qf_amazon_alexa" example specific to QuickLogic

5. Exit MINGW32 and reopen again for the script file to be automatically executed

6. Then build and flash the image to DevKiTC-VE

   $ cd esp-va-sdk/examples/
   $ make menuconfig
      => Select Partition table > Single factory and save and exit
   $ make flash   
     (optional: make -j 8 flash, where -j 8 instruct the make to spawn 8 threads to speed the compilation)
   
   You should see build taking place followed by Linking.
   Then it will open the COM port and flash the new image

7. Type the following command to execute it.

   $ make monitor 

   You should see some debug messages followed by a message like this
   
   -----------------------------------------------------------
   I (xxx) conn_mgr_prov: Provisioning started with :
     service name = ESP-Alexa-xxxx
     service key =
     proof of possession (pop): abcd1234
   -----------------------------------------------------------
   
   See the following link for more info.
   https://github.com/espressif/esp-va-sdk/blob/master/examples/amazon_alexa/README-Alexa.md
   
   At this point the Device is not provisioned.
   
8. The Device can be provisioned using "ESP Alexa" App from a Andriod Phone
   Provisioning means - setting your Wifi network credentials and an  
   Amazon login Credentials.
   
   Note: Do not use your Personal Amazon Account. Create one if needed for testing.
   
   First install "ESP Alexa" App on an Andriod Phone 
   https://play.google.com/store/apps/details?id=com.espressif.provbleavs
   
   Start the "ESP Alexa" app and follow the guidlines.
   (The above webpage page has 5 screens that show what to do)
   - Frist select your Device from the list
   - Login into the Test Amazon Account created.
   - Select your Wifi network and provide the password
   
   Then You should see the device connecting to the Wifi network and print
   ============= Alexa Ready================
   
   The Wifi and Amazon account credentials will be stored in the Flash.
   So, you need do that every time you start the Board.
   
   If you need to re-provision and delete the old data, give the following command
    
   $ make erase_flash

9. Connect the QuickFeather Board to DevKitC-VE Board

10. Connect a DAC Board to DevKitC-VE Board


   


    
