# Changelog
All notable changes to the Helen firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [3.1.1] - 2024-06-20
### Fixed
- some minor bugs

### Known Issues
- unstable regulator at low currents -> visible flickering
- voltage limiter has delay
- temperature limiter overshoots

## [3.1.0] - 2024-03-12
### Added
- override mode (immediately see changes while configure modes in the app)

### Known Issues
- unstable regulator at low currents -> visible flickering
- voltage limiter has delay
- temperature limiter overshoots

## [3.0.0] - 2024-02-24
### Added
- dedicated service for helen project and kd2 hardware configuration
- support for changing device name
- added off mode

### Changed
- advertising and scanning reworked, whitelist is only used for scanning, advertising is handled manually

### Deprecated
- the helena configuration service will be first reduced to remote control functions and later completely removed

### Fixed
- noisy temperature measurement in idle mode
- missing report handler for factory reset
- fixed rounding error for channel angle offset
- unnecessary power field in measurement data while light is off

### Known Issues
- unstable regulator at low currents -> visible flickering
- voltage limiter has delay
- temperature limiter overshoots

## [v2-alpha] - 2022-11-21
### Added

### Changed

### Removed

### Fixed

### Known Issues
- unstable regulator at low currents -> visible flickering
- voltage limiter has delay
- temperature limiter overshoots
