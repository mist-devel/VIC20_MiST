# VIC-20 for [MIST Board](https://github.com/mist-devel/mist-board/wiki)

### Features:
- Based on [The Replay Board](http://www.fpgaarcade.com/kb/commodore-vic-20/) code
- 3k RAM Expansion at $0400
- 8k/16k/24k RAM packs from $2000
- Read-only or writeable CART memory at $a000
- Joystick support
- PAL Display (with optional scandoubler for VGA monitors)
- PRG/CART loading
- 1541 Disk Drive support

### ROM update:
The built-in ROMs can be overridden via placing a vic20.rom file to the root of the SD Card.
The file format is: 1541 (16k) + Kernal (8k) + Basic (8k) + Char (4k)
