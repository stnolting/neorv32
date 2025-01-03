import sys
from usb_iss import UsbIss, defs
import time
import argparse
from argparse import ArgumentDefaultsHelpFormatter


class Interface:
    """Interface for I2C Memory"""

    # bootloader.c notation
    EXE_OFFSET_SIZE = 4
    EXE_OFFSET_DATA = 12

    def __init__(self, port, slave_id, max_bytes_size, two_address_bytes):
        # Test for Value Errors
        if not (0 <= slave_id <= 127):
            raise ValueError(
                f"Not fulfilled: 0 <= slave_id({slave_id}) <= 127")
        self._iss = UsbIss()
        self._iss.open(port)
        self._iss.setup_i2c()
        self._slave_id = slave_id
        self._max_bytes_size = max_bytes_size
        self._two_address_bytes = two_address_bytes

    def _i2c_write(self, i2c_slave_id, register, data):
        if self._two_address_bytes:
            self._iss.i2c.write_ad2(i2c_slave_id, register, data)
        else:
            self._iss.i2c.write_ad1(i2c_slave_id, register, data)

    def _i2c_read(self, i2c_slave_id, register, byte_count):
        if self._two_address_bytes:
            return self._iss.i2c.read_ad2(i2c_slave_id, register, byte_count)
        else:
            return self._iss.i2c.read_ad1(i2c_slave_id, register, byte_count)
            
    def _split_addr(self, address):
        if self._two_address_bytes:
            # Pseudo offset and one 16-bit block
            return 0, address & 0xFFFF
        else:
            # Split into two 8-bit blocks (though the first one actually cannot be that high)
            return (address >> 8) & 0xFF, address & 0xFF

    def write_bin_file(self, file_path):
        """Write a binary file byte-wise to slave until last Slave ID. Fill-in with 0x00."""
        with open(file_path, 'rb') as f:
            # Read the entire file content
            bin_data = f.read()
        if len(bin_data) > (self._max_bytes_size):
            raise RuntimeError(f"File too big {len(bin_data)} > {self._max_bytes_size} [bytes]")
        print("Write size: ", len(bin_data), " bytes")
        # New array with filled up 0x0
        write_data = bin_data + b'\x00' * (self._max_bytes_size - len(bin_data))
        print("Write...", end="")
        sys.stdout.flush()
        for i in range(0, len(write_data), 8): # Write in block of 8 bytes
            block = write_data[i:i + 8]
            address_blocks = self._split_addr(i)
            self._i2c_write(address_blocks[0] + self._slave_id, address_blocks[1] , list(block))
            time.sleep(0.01)
        print("done")

    def read_bin_file(self):
        """Read image from memory"""
        size_bytes = self._i2c_read(self._slave_id, self.EXE_OFFSET_SIZE, 4)
        size = (size_bytes[3] << 24) | (size_bytes[2] << 16) | (size_bytes[1] << 8) | size_bytes[0]
        size += self.EXE_OFFSET_DATA # Add padding for executable header
        if size > (self._max_bytes_size):
            raise RuntimeError(f"Noted size too big {size} > {self._max_bytes_size} [bytes]")
        print("Read size: ", size, " bytes")
        print("Read...", end="")
        read_data = b''
        for i in range (0, size, 32): # Read in max. block of 32 bytes
            byte_count = max(32, i - size) # Last block may be smaller
            address_blocks = self._split_addr(i)
            read_data += bytes(self._i2c_read(address_blocks[0] + self._slave_id, address_blocks[1], byte_count))

        print("done")
        return read_data
    
    def validate_bin_file(self, file_path):
        """Compare read file and given file"""
        with open(file_path, 'rb') as f:
            # Read the entire file content
            reference_data = f.read()
        read_data = self.read_bin_file()
        read_size = len(read_data)
        reference_size = ((len(read_data) + 31) // 32) * 32 # Round up to next multiple of 32
        assert reference_size == read_size, f"Read size {read_size} != {len(reference_data)} (32 bit aligned) [bytes]"
        for i in range(0, len(reference_data)): # Compare only to data content
            assert reference_data[i] == read_data[i], f"Read data {read_data[i]} != {reference_data[i]} at address {i}"
        print("Validation successful")

if __name__ == "__main__":
    # Argparse
    parser = argparse.ArgumentParser(
        description="Python script to upload program to I2C Memory via USB ISS.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "file", type=str,
        help="Path to binary file, read if no -s, write with -s")
    parser.add_argument(
        "-p", "--port", type=str,
        default="/dev/ttyACM0",
        help="I2C ISS Port")
    parser.add_argument(
        "-s", "--save", action='store_true',
        default=False,
        help="Readout memory and safe to file")
    parser.add_argument(
        "-i", "--slave-id", type=int,
        default=0x50,
        help="First I2C Slave ID")
    parser.add_argument(
        "-m", "--max-bytes-size", type=int,
        default=2048,
        help="Size of memory in bytes")
    parser.add_argument(
        "-t", "--two-address-bytes", action='store_true',
        default=False,
        help="Enables two-bytes-addressing")
    args = parser.parse_args()

    ifc = Interface(args.port, args.slave_id, args.max_bytes_size, args.two_address_bytes)
    if(args.save):
        with open(args.file, "wb") as f:
            f.write(ifc.read_bin_file())
    else:
        ifc.write_bin_file(args.file)
        ifc.validate_bin_file(args.file)