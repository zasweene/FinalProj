# Final Project

This solution is built around the 'rx_sniff.c' file. All of the file dependencies are set up for the compilation of 'rx_sniff.c'.

## Flashing New Programs

To flash a new program to the board:

1. **Open 'rx_sniff.c'**
2. **Replace its contents** with the new program code.
3. **Compile and flash** the modified 'rx_sniff.c' using nRF connect.

This method avoids modifying the build scripts.

## Included Programs

This repository includes two programs that can be flashed to the board by copying their contents into 'rx_sniff.c':

- **`Receiver.c`** – Receives a `.png` file from a sender.
- **`Sender.c`** – Sends a `.png` file to a receiver.
