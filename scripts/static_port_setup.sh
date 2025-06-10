#!/bin/bash

PORT=$1
SYMLINK=$2
RULES_DIR="/etc/udev/rules.d/99-usb-serial.rules"


echo ""

if [ "$EUID" -ne 0 ]; then
    printf "Please run as root (sudo).\n\n"
    exit 1
fi

if [ $# -ne 2 ]; then
    printf "Missing input arguments:\n"
    echo "1. serial port name (e.g. ttyUSB0 (no need absolute path))"
    printf "2. SYMLINK name of static port\n\n"
    exit 1
fi

if [ -e ${SYMLINK} ]; then
    echo "${SYMLINK} already exists"
    echo "If issue persists, run:"
    printf "\n'sudo udevadm control --reload && sudo udevadm trigger'\n\nthen re-run the script.\n\n"
    exit 1
fi

if [  ! -e $PORT ]; then
    printf "Serial port $PORT not found!\n\n"
    exit 1
fi

info=$(udevadm info --name=$PORT --attribute-walk | awk 'BEGIN { RS="\n\n"; FS="\n" } NR==3 {print} NR==6 {exit}')
phys=$(echo "$info" | grep phys -m 1 | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
SUBSYSTEM=$(echo "$info" | grep SUBSYSTEM -m 1 | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
name=$(echo "$info" | grep name -m 1 | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
uniq=$(echo "$info" | grep uniq -m 1 | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')

MAJOR=$(udevadm info $PORT | grep MAJOR | cut -d "=" -f 2)
MINOR=$(udevadm info $PORT | grep MINOR | cut -d "=" -f 2)

# For resetting the port later
idVendor=$(udevadm info --name=$PORT --attribute-walk | awk 'BEGIN { RS="\n\n"; FS="\n" } NR==7 {print} NR==10 {exit}' | grep idVendor)
idProduct=$(udevadm info --name=$PORT --attribute-walk | awk 'BEGIN { RS="\n\n"; FS="\n" } NR==7 {print} NR==10 {exit}' | grep idProduct)
vendorid=$(echo "$idVendor" | sed 's/[^"]*"\([^"]*\)".*/\1/')  
productid=$(echo "$idProduct" | sed 's/[^"]*"\([^"]*\)".*/\1/')
echo "vendor ID: $vendorid | product ID: $productid"

text="$SUBSYSTEM, $phys, $name, $uniq, MODE:=\"0666\", ENV{MAJOR}==\"$MAJOR\", ENV{MINOR}==\"$MINOR\" SYMLINK+=\"${SYMLINK}\""
text=$(echo "$text" | sed 's/,\s\{0,1\}\([^,]\)/,\1/g')  # Remove spaces in case the above attributes are missing

matching_text=$(cat "$RULES_DIR" 2> /dev/null | grep "$text")
if [ -n "$matching_text" ]; then
    printf "$SYMLINK: already has an existing static port with same name.\n\n"
else
    echo "Adding new rule to $RULES_DIR"
    echo "# Device for $SYMLINK" >> "$RULES_DIR"
    echo "$text" >> "$RULES_DIR"
    echo "" >> "$RULES_DIR"
    printf "Added $SYMLINK for $(echo "$product" | sed 's/[^"]*"\([^"]*\)".*/\1/')\n\n"
fi

sudo udevadm control --reload
printf "Resetting $PORT with $vendorid and $productid...\n\n"
sudo usb_modeswitch -v "0x$vendorid" -p "0x$productid" --reset-usb 1> /dev/null

if [ -n "$matching_text" ]; then
    exit 0
fi

matching_text=$(cat "$RULES_DIR" 2> /dev/null | grep "$text")
if [ -n "$matching_text" ]; then
    echo "New static port: $SYMLINK"
else
    echo "ERROR: $SYMLINK was not assigned into text"
fi

echo ""
exit 0
