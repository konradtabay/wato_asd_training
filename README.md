# WATonomous ASD Admissions Assignment

Fork of [WATonomous/wato_asd_training](https://github.com/WATonomous/wato_asd_training) for local development.

Official assignment: [WATonomous wiki](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/)

## Quick start

1. Install Docker (Engine on Linux/WSL, Desktop on macOS).
2. Clone this repo and follow **[SETUP.md](SETUP.md)** for `watod-config.local.sh`, build, Foxglove, and Apple Silicon notes.

```bash
git clone https://github.com/konradtabay/wato_asd_training.git
cd wato_asd_training
cp watod-config.local.sh.example watod-config.local.sh
# Edit PLATFORM / ACTIVE_MODULES if needed — see SETUP.md
./watod build
./watod up
```

## Prerequisites (upstream)

Supported hosts: Linux Ubuntu 22.04+, Windows (WSL), macOS. ROS is not installed on the host; everything runs in Docker via `watod`.
