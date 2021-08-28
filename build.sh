#!/bin/bash

rm ./Dynamixel-XL320.zip
zip -r Dynamixel-XL320.zip . -x "*.git*" -x "*.vscode*" -x "*.DS_Store*" -x "build.sh" -x "Dynamixel-XL320.ino"
