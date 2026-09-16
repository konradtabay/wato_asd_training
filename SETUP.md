# Local setup (ASD admission assignment)

Assignment writeup: [WATonomous wiki](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/)  
Infrastructure: [Monorepo infrastructure](https://wiki.watonomous.ca/autonomous_software_general/monorepo_infrastructure/)

## Prerequisites

- Linux (Ubuntu 22.04+), Windows (WSL), or macOS
- [Docker Engine](https://docs.docker.com/engine/install/) or Docker Desktop
- Git

## Clone

```bash
git clone https://github.com/konradtabay/wato_asd_training.git
cd wato_asd_training
```

Optional upstream remote (to pull WATonomous updates):

```bash
git remote add upstream https://github.com/WATonomous/wato_asd_training.git
git fetch upstream
```

## Local watod config

`watod-config.local.sh` is gitignored. Create it from the example:

```bash
cp watod-config.local.sh.example watod-config.local.sh
```

Edit as needed:

| Variable | Linux / WSL (typical) | macOS Apple Silicon |
|---|---|---|
| `ACTIVE_MODULES` | `"robot gazebo vis_tools"` | same |
| `PLATFORM` | omit (defaults to `amd64`) | `"arm64"` |

## Build and run

```bash
./watod build
./watod up
```

First `./watod build` on Apple Silicon can take 20–40 minutes.

To rebuild after changing robot code:

```bash
./watod down robot
./watod build robot
./watod up robot
```

Stop everything:

```bash
./watod down
```

## Foxglove

1. Install [Foxglove Desktop](https://foxglove.dev/download) or open [app.foxglove.dev](https://app.foxglove.dev).
2. **Open connection** → **Foxglove WebSocket** (not the browser address bar).
3. Port is assigned on first `./watod` run. Check `modules/.env`:

   ```bash
   grep FOXGLOVE_BRIDGE_PORT modules/.env
   ```

   Example: `FOXGLOVE_BRIDGE_PORT=10020` → connect to `ws://localhost:10020`

4. **Layout** → **Import layout** → select:

   ```
   config/wato_asd_training_foxglove_config .json
   ```

   The filename has a **space** before `.json`.

5. Warmup: add a **Raw Messages** panel on `/test_topic` to see `"Hello, ROS 2!"` every 500 ms.

## Apple Silicon notes

This fork includes Dockerfile changes for `humble-arm64` base images:

- Refresh expired ROS apt signing keys
- Pin `ros-humble-rosbag2` instead of a `rosbag2*` glob (avoids 404 debug-symbol packages)

Without these, `./watod build` fails on arm64 Macs with apt key / 404 errors.

## Dev environment (IntelliSense)

```bash
./watod --setup-dev-env robot
```

Follow the terminal prompts for VS Code integration.
