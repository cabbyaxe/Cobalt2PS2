# Cobalt2PS2
Enables raspberry pi pico to act as a cobalt flux control box for PS2 games, reading the state of the cobalt flux dance pads and translating those into PS2 button presses
Up, Down, Left, and Right translate to Up, Down, Left, and right on the PS2 controller. Top left translates to "O" and top right translates to "X" on the controller.

## Setup

Cut an old PS2 controller cable, and get a DB-15 connector for the cobalt flux dance pad like [this one](https://www.amazon.com/dp/B07F9S61QT)

PS2 controller pin information is found here: [PSX-SPX](https://psx-spx.consoledev.net/pinouts/#controller-ports-and-memory-card-ports)
<img src="https://psx-spx.consoledev.net/controller-pinout.jpg" alt="PS2 controller pinout">

Solder the GND pins from the Cobalt Flux dance pad and PS2 controller to GND pins on the pico
Solder your PICO GPIO pins as follows (This assumes a Cobalt Flux dance pad with a DB-15 connector):

| GPIO Pin  | Second Header |
| ------------- | ------------- |
| 5  | PS2 DAT  |
| 6  | PS2 CMD  |
| 7  | PS2 SEL  |
| 8  | PS2 CLK  |
| 9  | PS2 ACK  |
| 0  | Cobalt Flux pin 2 Up  |
| 1  | Cobalt Flux pin 3 Down  |
| 2  | Cobalt Flux pin 4 Left  |
| 3  | Cobalt Flux pin 5 Right  |
| 4  | Cobalt Flux pin 6 Up Left  |
| 10  | Cobalt Flux pin 7 Up Right  |
| 11  | Cobalt Flux pin 8 Down Left  |
| 12  | Cobalt Flux pin 9 Down Right  |



Compile this project using the raspberry pi pico VSCode extension and flash the UF2 file to the pico (Or flash the compiled UF2 file in the releases section).


### Acknowledgments

This project utilizes the `psxSPI.pio` file from the [PicoMemcard](https://github.com/dangiu/PicoMemcard) project

## License

This project is licensed under the [GNU Public License Version 3](LICENSE).


