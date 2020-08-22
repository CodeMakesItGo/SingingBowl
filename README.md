# SingingBowl
Application code for the Singing Bowl Project

## What is the SiningBowl?
During the corona lock-down I've "slowly" realized that having time to yourself is very important. I think this is true if you are introverted or not. I am an introvert and being around family all day every day for weeks is, even though this sounds horrible, STRESSFUL!

I want to build a device that helps and promote meditation using singing bowl therapy. If you haven't heard a signing bowl, they produce an amazing sound that reverberates throughout your body. Just listening to signing bowl(s) for a couple seconds can calm you down a lot. Unfortunately, the same effect cannot be felt through listening to a recording. The vibrations the bowls create are just…different. I'm calling this device the Singing Bowl Player (SBP) because I'm super creative in thinking about names.

The player isn't physically complex, it is basically a one track record player that is spinning a heavy bowl instead of a flat record. My current plan is that it kind of looks like the Kitchen-aid blender.

This project was entered in the "Change the Planet with PSoC® IoT Design Contest". Check out the project here: https://hackaday.io/project/171713-singing-bowl-player


## Arduio Setup
The Arduio code is developed for the Wemos D1 Lite. This AWS IOT portion of this code originated from these instructions:
https://electronicsinnovation.com/how-to-connect-nodemcu-esp8266-with-aws-iot-core-using-arduino-ide-mqtt/

## PSoC Setup
The PSoC code was developed for the CY8CKIT-062-WiFi-BT. The PSoC setup instruction for the AWS IOT portion of the code originates from these instructions:
https://github.com/cypresssemiconductorco/afr-example-capsense

This way, I could get the exampl running quickly for the CapSense buttons and modify the code slightly for my specific topic. The modified code is contained in this repo. After following the instructions from the link above, replace the code in the '\amazon-freertos\projects\cypress\afr-example-capsense\source' director with the files contained in this repo 'PSoC\Source'.
