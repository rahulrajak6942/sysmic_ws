# Environment Setup

This document highlights the prerequisites needed for running the container smoothly.
The environment allows usage of ROS2 nodes with the host computer without executing commands in the container repeatedly. 

---

## Docker Installation

1. Set up Docker's `apt` repository.
    ```bash
    # Add Docker's official GPG key:
    sudo apt update
    sudo apt install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
    Types: deb
    URIs: https://download.docker.com/linux/ubuntu
    Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
    Components: stable
    Architectures: $(dpkg --print-architecture)
    Signed-By: /etc/apt/keyrings/docker.asc
    EOF

    sudo apt update
    ```

2. Install the Docker packages.
    ```bash
    sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

    1. Verify that Docker is running.
        ```bash
        sudo systemctl status docker
        ```
    2. If Docker is not running, start it manually.
        ```bash
        sudo systemctl start docker
        ```

3. Verify that the installation is successful.
    ```bash
    sudo docker run hello-world
    ```

---

## Post-installation

1. Create the `docker` group.
    ```bash
    sudo groupadd docker
    ```

2. Add your user to the `docker` group.
    ```bash
    sudo usermod -aG docker $USER
    ```

3. Log out and log back in so that your group membership is re-evaluated.

4. Verify that you can run `docker` commands without `sudo`.
    ```bash
    docker run hello-world
    ```

5. Grant X server access to local container clients.
    ```bash
    xhost +local:docker
    ```

---

## GPU Deployment

1. Configure the repository.
    ```bash
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && sudo apt-get update
    ```

2. Install the NVIDIA Container Toolkit packages.
    ```bash
    sudo apt-get install -y nvidia-container-toolkit
    sudo systemctl restart docker
    ```

3. Configure the container runtime.
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```

4. Verify NVIDIA Container Toolkit.
    ```bash
    docker run --rm --runtime=nvidia --gpus all nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
    ```

---