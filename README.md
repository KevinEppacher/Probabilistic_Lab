
# Project Repository Guide

Welcome to the project repository! This guide will help you get started with cloning the package, setting up Docker, understanding what's included in this workspace, and locating the README for the particle filter.

## Cloning the Package

To clone this package, use the following command:

```bash
git clone --recurse-submodules <Repository-URL>
```

Replace `<Repository-URL>` with the actual URL of the repository. This command ensures that you also clone any submodules associated with the repository.

## Setting Up Docker

There are two main ways to set up Docker for this project: using Visual Studio Code Remote - Containers extension or using Docker commands directly.

### Using VS Code Remote - Containers

1. Ensure you have Docker installed and running on your system.
2. Install Visual Studio Code and the Remote - Containers extension.
3. Open the project folder in VS Code.
4. Use the command palette (Ctrl+Shift+P or Cmd+Shift+P on macOS) and select "Remote-Containers: Reopen in Container". This will build and start the Docker container based on the configuration found in the project.

### Using Docker Commands

If you prefer using Docker commands directly, follow these steps:

1. Build the Docker image:
    ```bash
    docker build -t project-image .
    ```
2. Run the Docker container:
    ```bash
    docker run -d --name project-container project-image
    ```

Replace `project-image` with a name for your Docker image and `project-container` with a name for your Docker container.

## Workspace Contents

This workspace includes:

- Source code for the main project.
- A Dockerfile for setting up the development environment.
- Submodules for any dependencies or related projects.
- Documentation and guides, including this README.

## Particle Filter README

For detailed information about the particle filter implementation, please refer to the README located at:

`/src/particle_filter/README.md`

This document provides an overview of the particle filter, including its setup, configuration, and usage instructions.

## Teaser

![Particle Filter Demo](src/sensor_fusion/docs/Demo_Particle_Filter.gif)

