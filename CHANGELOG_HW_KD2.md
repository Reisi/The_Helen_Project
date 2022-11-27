# Changelog
All notable changes to the KD2 board hardware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [1.1] - 2022-11 - Unreleased
### Added
- green status led

### Changed
- pin rearrangement, COM pin is now on analog pin

### Removed
- INT2 pin of BMI160 not needed

### Fixed
- SD0 pin of BMI160 connected to GND

### Known Issues

## [1.0] - 2022-02
### Added

### Changed

### Removed

### Fixed

### Known Issues
- SD0 pin of BMI160 floating, needs to be connected to GND
- protection diode (D5) on COM pin should be replaced with higher voltage type
  to use pin as an open drain PWM signal output
  -> fixed, all board are/will be assembled with 18V zener
- current measurement resistor (R6) needs to be replaced with 680 Ohm, 820 Ohm
  is too close to adc measurement limits
  -> fixed, all boards are/will be reassembled with 680 Ohm resistor
- onboard antenna not working when case closed and no debugging cable connected
  -> remove onboard antenna and add external
