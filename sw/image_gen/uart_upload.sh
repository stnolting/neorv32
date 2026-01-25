#!/usr/bin/env bash

# Simple script to upload an executable to the default NEORV32 bootloader

set -e

# Default baud rate
BAUD=19200

# check arguments; show help text
if [ $# -ne 2 ]
then
  printf "Upload an executable via serial port (UART) to the NEORV32 bootloader.\n"
  printf "If the upload is successful, the executable will be run automatically.\n"
  printf "Reset processor before starting the upload.\n\n"
  printf "Usage:   ./uart_upload.sh <serial port> <NEORV32 executable>\n"
  printf "Example: ./uart_upload.sh /dev/ttyS6 path/to/neorv32_exe.bin\n"
  exit 0
fi

# timeout to make sure there is enough time for the file to be sent in background
# "BAUD" bits per second; 10 bits per byte (incl START and STOP bits); plus safety margin
FILESIZE=$(stat -c%s "$2")
TIMEOUT=$(( (FILESIZE / (BAUD/10)) + 2 ))

# setup serial port
printf "Opening serial port ($1)... "
stty -F $1 $BAUD raw -echo -ixon -ixoff -icrnl -onlcr
exec 3<>$1
printf "OK\n"

# send executable
printf "Uploading executable ($FILESIZE bytes)... "
printf " " >&3 # skip auto-boot
printf "u" >&3 # start upload
cat $2 >&3     # send executable

# wait for upload to complete and check bootloader response
UPOLOAD_OK=0
EXPIRED=$(( SECONDS + TIMEOUT ))
PATTERN="OK"
BUFFER=""
while [ $SECONDS -lt $EXPIRED ]; do
  if read -r -n 1 -u 3 -t 1 ch; then
    BUFFER+="$ch"
    UPOLOAD_OK=-1
    if [[ "$BUFFER" == *"$PATTERN"* ]]; then
      UPOLOAD_OK=1
      break
    fi
  fi
done

if [ $UPOLOAD_OK -eq 1 ]; then
  printf "OK\n" >&2
elif [ $UPOLOAD_OK -eq -1 ]; then
  printf "\nERROR! Invalid bootloader response.\n" >&2
else
  printf "\nERROR! Bootloader response timeout ($TIMEOUT seconds).\n" >&2
fi

# start executable
if [ $UPOLOAD_OK -eq 1 ]; then
  printf "Booting executable...\n"
  printf "e" >&3
fi

# done
exec 3>&-
if [ $UPOLOAD_OK -eq 1 ]; then
  exit 0
else
  printf "Check configuration, reset processor and try again.\n" >&2
  exit 1;
fi
