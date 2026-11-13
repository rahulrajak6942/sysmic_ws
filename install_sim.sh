#!/usr/bin/env bash
# One-command setup for the SYSMIC V2 simulation (Ubuntu 22.04).
#
#   cd ~/sysmic_ws
#   bash install_sim.sh
#
# Installs, only if missing: Miniconda, a "robocasa" conda env (Python 3.11),
# robosuite + RoboCasa (in ~/robocasa_ws) and the kitchen assets (about 10 GB).
# Then it runs the SYSMIC self-test and builds this ROS 2 workspace.
# Safe to run again: finished steps are skipped.
set -euo pipefail

ENV_NAME=robocasa
WS="$HOME/robocasa_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say() { printf '\n==> %s\n' "$*"; }

# ---- 1. Miniconda -----------------------------------------------------------
if command -v conda >/dev/null 2>&1; then
    CONDA_BASE="$(conda info --base)"
elif [ -x "$HOME/miniconda3/bin/conda" ]; then
    CONDA_BASE="$HOME/miniconda3"
elif [ -x "$HOME/miniforge3/bin/conda" ]; then
    CONDA_BASE="$HOME/miniforge3"
elif [ -x "$HOME/anaconda3/bin/conda" ]; then
    CONDA_BASE="$HOME/anaconda3"
else
    say "Installing Miniconda into ~/miniconda3"
    wget -q --show-progress -O /tmp/miniconda.sh \
        https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
    rm -f /tmp/miniconda.sh
    CONDA_BASE="$HOME/miniconda3"
    "$CONDA_BASE/bin/conda" init bash >/dev/null
fi
set +u  # conda's own shell scripts read variables that may be unset
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"

# Keep conda out of normal terminals so it cannot shadow the Python that ROS 2 uses.
conda config --set auto_activate_base false >/dev/null 2>&1 \
    || conda config --set auto_activate false >/dev/null 2>&1 || true
# Newer conda refuses to create envs until the default channels' terms are accepted.
for ch in https://repo.anaconda.com/pkgs/main https://repo.anaconda.com/pkgs/r; do
    conda tos accept --override-channels --channel "$ch" >/dev/null 2>&1 || true
done

# ---- 2. Python 3.11 environment --------------------------------------------
if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    say "Conda env '$ENV_NAME' already exists"
else
    say "Creating conda env '$ENV_NAME' (Python 3.11)"
    conda create --override-channels -c conda-forge -n "$ENV_NAME" python=3.11 tk -y
fi
conda activate "$ENV_NAME"
set -u
PYV="$(python -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
if [ "$PYV" != "3.11" ]; then
    echo "ERROR: env '$ENV_NAME' has Python $PYV, expected 3.11. Remove it with:"
    echo "       conda env remove -n $ENV_NAME    and run this script again."
    exit 1
fi

# ---- 3. robosuite + RoboCasa ------------------------------------------------
mkdir -p "$WS"
if [ ! -d "$WS/robosuite/.git" ]; then
    say "Cloning robosuite"
    git clone https://github.com/ARISE-Initiative/robosuite "$WS/robosuite"
fi
if [ ! -d "$WS/robocasa/.git" ]; then
    say "Cloning RoboCasa"
    git clone https://github.com/robocasa/robocasa "$WS/robocasa"
fi
if python -c "import robosuite, robocasa" >/dev/null 2>&1; then
    say "robosuite and RoboCasa already installed"
else
    say "Installing robosuite and RoboCasa (several minutes)"
    pip install -e "$WS/robosuite"
    pip install -e "$WS/robocasa"
fi

# ---- 4. Kitchen assets ------------------------------------------------------
RC_DIR="$(python -c 'import robocasa; print(robocasa.__path__[0])')"
if [ -f "$RC_DIR/macros_private.py" ]; then
    say "Macros already set up"
else
    say "Setting up macros"
    python -m robocasa.scripts.setup_macros
fi
ASSETS="$RC_DIR/models/assets"
have_assets=yes
for d in textures generative_textures objects/objaverse objects/aigen_objs objects/lightwheel; do
    [ -d "$ASSETS/$d" ] || have_assets=no
done
if [ "$have_assets" = yes ]; then
    say "Kitchen assets already downloaded"
else
    say "Downloading kitchen assets (about 10 GB). If it stops, run this script again."
    # the downloader asks "Proceed? (y/n)"; answer yes
    python -m robocasa.scripts.download_kitchen_assets < <(printf 'y\n%.0s' 1 2 3 4 5 6 7 8 9 10)
fi

# ---- 5. SYSMIC self-test -----------------------------------------------------
say "Running the SYSMIC self-test in a RoboCasa kitchen"
python "$HERE/src/sysmic_sim/sim/sysmic_robosuite.py" --no-viewer

# ---- 6. Build the ROS 2 workspace -------------------------------------------
# colcon must run on the system Python, so this happens outside the conda env.
built=no
if [ -f /opt/ros/humble/setup.bash ]; then
    say "Building the ROS 2 workspace"
    if (
        set +eu
        conda deactivate
        conda deactivate
        source /opt/ros/humble/setup.bash
        cd "$HERE" && colcon build --symlink-install
    ); then
        built=yes
    else
        echo "colcon build failed. Build it by hand (see below)."
    fi
else
    echo "ROS 2 Humble not found in /opt/ros/humble; skipping the workspace build."
fi

cat <<EOF2

Done. Open a NEW terminal (no conda env active) and run:

    source /opt/ros/humble/setup.bash
    cd "$HERE"
EOF2
[ "$built" = yes ] || echo "    colcon build --symlink-install"
cat <<EOF2
    source install/setup.bash
    ros2 launch sysmic_sim sim.launch.py
EOF2
