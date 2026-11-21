# ESP32-S3-ePaper-1.54 Example

Example project for [Waveshare ESP32-S3-ePaper-1.54](https://www.waveshare.com/wiki/ESP32-S3-ePaper-1.54) development board.

## Hardware

- **Board**: Waveshare ESP32-S3-ePaper-1.54
- **MCU**: ESP32-S3 with 4MB Flash + 2MB PSRAM
- **Panel**: GDEY0154D67 (200×200 pixels, Black/White)
- **Features**: Partial refresh support, RTC, Temperature sensor

## Pinout

| Signal | GPIO |
|--------|------|
| BUSY   | 8    |
| RST    | 9    |
| DC     | 10   |
| CS     | 11   |
| SCK    | 12   |
| MOSI   | 13   |
| PWR_EN | 6    |

## Build and Flash

```bash
# Set target
idf.py set-target esp32s3

# Build
idf.py build

# Flash and monitor
idf.py -p PORT flash monitor
```

## Features Demonstrated

- E-paper initialization with `EPD_CONFIG_ESP32S3_154()` preset
- LVGL 9 integration
- **Vietnamese text** with FreeType-hinted fonts (Be Vietnam Pro Medium, 12/16 px)
- **Floyd-Steinberg dithering** for grayscale simulation on B/W panel, with
  `dither_protect_bw` so 1 bpp text and lines stay crisp
- **Optional 4-gray mode** (`idf.py menuconfig` -> E-Paper 1.54 Example -> Use 4-gray mode)
- Partial refresh for fast updates
- Periodic full refresh to clear ghosting
- Board power control (GPIO6)

## Demo UI

The example displays:

1. **8-level grayscale bar** - Demonstrates dithering quality with discrete gray steps
2. **Continuous gradient** - Shows smooth black-to-white transition using dithering
3. **Gray boxes** - 4 boxes with gray levels 0, 85, 170, 255 (exact levels in 4-gray mode)
4. **Counter** - Updates every 10 seconds using partial refresh

## Usage

The example displays a counter that updates every 10 seconds using partial refresh. Every 5 updates, a full refresh is performed to clear any ghosting artifacts.

The grayscale dithering uses Floyd-Steinberg error diffusion to simulate gray levels on the B/W panel. This creates the illusion of more than 2 colors by varying the density of black pixels.

## Fonts

| File | Mode | Generated with |
|------|------|----------------|
| `main/font_vn_12.c`, `main/font_vn_16.c` | B/W | 1 bpp, FreeType mono hinting |
| `main/font_vn_12_aa.c`, `main/font_vn_16_aa.c` | 4-gray | 2 bpp, light hinting |

Regenerate them with `./gen_fonts.sh` (needs `pip install -r ../../tools/font_conv/requirements.txt`).
The source font is `fonts/BeVietnamPro-Medium.ttf` (SIL Open Font License, see `fonts/OFL.txt`).

## License

MIT License
