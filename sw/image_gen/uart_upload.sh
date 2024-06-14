#!/usr/bin/env bash

set -e

# Simple script to upload executable via bootloader

if [ $# -ne 2 ]
then
  printf "Upload and execute application image via serial port (UART) to the NEORV32 bootloader.\n"
  printf "Reset processor before starting the upload.\n\n"
  printf "Usage:   sh uart_upload.sh <serial port> <NEORV32 executable>\n"
  printf "Example: sh uart_upload.sh /dev/ttyS6 path/to/project/neorv32_exe.bin\n"
  exit 0
fi

# configure serial port
stty -F "$1" 19200 -hup raw -echo -echoe -echok -echoctl -echoke -ixon cs8 -cstopb noflsh clocal cread

# abort autoboot sequence
printf " " > $1 # send any char that triggers no command

# execute upload command and get response
exec 3<$1                            # redirect serial output to fd 3
cat <&3 > uart_upload.response.tmp & # redirect serial output to file
PID=$!                               # save pid to kill cat later
printf "u" > $1                      # send upload command to serial port
sleep 0.5s                           # wait for bootloader response
kill $PID                            # kill cat process

exec 3<&- # free fd 3

# check response
if ! grep -Fq "Awaiting neorv32_exe.bin" uart_upload.response.tmp;
then
  printf "Bootloader response error!\n"
  printf "Reset processor before starting the upload.\n"
  rm -f uart_upload.response.tmp
  exit 1
fi

# send executable and get response
printf "Uploading executable..."
exec 3<$1                            # redirect serial output to fd 3
cat <&3 > uart_upload.response.tmp & # redirect serial output to file
PID=$!                               # save pid to kill cat later
cat "$2" > "$1"                      # send executable to serial port
sleep 3s                             # wait for bootloader response
kill $PID                            # kill cat process

exec 3<&- # free fd 3

# check response
if ! grep -Fq "OK" uart_upload.response.tmp;
then
  printf " FAILED!\n"
  rm -f uart_upload.response.tmp
  exit 1
else
  printf " OK\n"
  echo "Starting application..."
  printf "e" > $1
  rm -f uart_upload.response.tmp
  exit 0
fi
