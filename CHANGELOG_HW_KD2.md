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
- just issue in schematic, 18V diode will be placed if available
  (protection diode (D5) on COM pin should be replaced with higher voltage type
  to use pin as an open drain PWM signal output)
