#!/usr/bin/env bash

set -e

# Simple script to upload executable to bootloader

if [ $# -ne 2 ]
then
  echo "Upload image via serial port (UART) to the NEORV32 bootloader."
  echo "Reset processor before starting the upload."
  echo "Usage:   [sudo] sh uart_upload.sh <port> <NEORV32 executable>"
  echo "Example: sh uart_upload.sh /dev/ttyS6 neorv32_exe.bin"
  exit
fi

# configure serial port
stty -F "$1" 19200 -hup raw -echo -echoe -echok -echoctl -echoke -crtscts cs8 -cstopb noflsh clocal cread

# trigger fast upload mode and get response
exec 3<$1                              # redirect serial output to fd 3
  cat <&3 > uart_upload.response.dat & # redirect serial output to file
  PID=$!                               # save pid to kill cat
    printf "u" > $1                    # send upload command to serial port
    sleep 0.5s                         # wait for bootloader response
  kill $PID                            # kill cat process

exec 3<&- # free fd 3

# check response
if ! grep -Fq "Awaiting neorv32_exe.bin" uart_upload.response.dat;
then
  echo "Bootloader response error."
  echo "Reset processor before starting the upload."
  rm -f uart_upload.response.dat
  exit
fi

# send executable and get repsonse
echo -n "Uploading... "
exec 3<$1                              # redirect serial output to fd 3
  cat <&3 > uart_upload.response.dat & # redirect serial output to file
  PID=$!                               # save pid to kill cat
    cat "$2" > "$1"                    # send executable to serial port
    sleep 3s                           # wait for bootloader response
  kill $PID                            # kill cat process

exec 3<&- # free fd 3

# check response
if ! grep -Fq "OK" uart_upload.response.dat;
then
  echo "Upload error."
  rm -f uart_upload.response.dat
  exit
fi

rm -f uart_upload.response.dat
echo "Done."
