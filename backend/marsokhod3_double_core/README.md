
# Overview

Intel Quartus Prime project for [Marsokhod3 board](http://marsohod.org/howtostart/plata-marsokhod3) ([shop](http://marsohod.org/shop/boards/brd-marsohod2150811130725150811131628)) with FPGA MAX10 10M50SAE144C8GES.

## Usage

Run in terminal from the project root directory:
```
  make Q_PROJECT=marsokhod3_double_core implement_mars3
  make Q_PROJECT=marsokhod3_double_core read_log_mars3
  make Q_PROJECT=marsokhod3_double_core prog_mars3_ram
```