<#
.SYNOPSIS
Helper script to build and launch ROS 2 Jazzy + Gazebo Harmonic with Hybrid GPU Rendering inside WSL2 Ubuntu 24.04 from Windows PowerShell.

.PARAMETER Task
The action to perform: 'launch' (default), 'rqt', 'standalone', 'build', 'topics', or 'bash'.
#>
param(
    [ValidateSet("launch", "rqt", "standalone", "build", "topics", "bash")]
    [string]$Task = "launch"
)

$Distro = "Ubuntu-24.04"
$GpuEnv = "export GALLIUM_DRIVER=d3d12 && export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA"

switch ($Task) {
    "launch" {
        Write-Host "Launching ROS 2 Surveillance Simulation (GUI with Hybrid GPU Rendering) via WSL2..." -ForegroundColor Cyan
        wsl -d $Distro --cd $PSScriptRoot bash -lc "$GpuEnv && source /opt/ros/jazzy/setup.bash && if [ ! -f install/setup.bash ]; then colcon build --symlink-install; fi && source install/setup.bash && ros2 launch surveillance_sim surveillance_world.launch.py"
    }
    "rqt" {
        Write-Host "Launching ROS 2 Simulation with RQt Camera Preview (Hybrid GPU Rendering) via WSL2..." -ForegroundColor Cyan
        wsl -d $Distro --cd $PSScriptRoot bash -lc "$GpuEnv && source /opt/ros/jazzy/setup.bash && if [ ! -f install/setup.bash ]; then colcon build --symlink-install; fi && source install/setup.bash && ros2 launch surveillance_sim surveillance_world.launch.py rqt:=true"
    }
    "standalone" {
        Write-Host "Launching Standalone Gazebo Harmonic World (Hybrid GPU Rendering) via WSL2..." -ForegroundColor Cyan
        wsl -d $Distro --cd $PSScriptRoot bash -lc "$GpuEnv && source /opt/ros/jazzy/setup.bash && gz sim -r src/surveillance_sim/worlds/surveillance_world.sdf"
    }
    "build" {
        Write-Host "Building ROS 2 workspace in WSL2..." -ForegroundColor Cyan
        wsl -d $Distro --cd $PSScriptRoot bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
    }
    "topics" {
        Write-Host "Listing active ROS 2 topics..." -ForegroundColor Cyan
        wsl -d $Distro --cd $PSScriptRoot bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list"
    }
    "bash" {
        Write-Host "Opening interactive bash shell with GPU acceleration configured ($Distro)..." -ForegroundColor Cyan
        wsl -d $Distro --cd $PSScriptRoot bash
    }
}
